#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "master.h"
#include "gpio.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"
#include "usart.h"

#define SIZE_STACK_SLAVE 256
#define STACK_SIZE_INTERN 512

typedef enum {KELVIN = 0, CELSIUS = 1} type_temp;
typedef enum {NO_LOG = 0, LOG     = 1} type_log;
typedef enum {ZERO_S = 0, TEN_S   = 1, THIRTY_S = 2, NINETY_S = 3} type_samp;

struct config {
    type_temp temperature;
    type_log log_time;
    type_samp samp_time;
    bool save_setting;
    uint8_t percentage;
} init = {
    .temperature = CELSIUS,
    .log_time = LOG,
    .samp_time = ZERO_S,
    .percentage = 10,
    .save_setting = true
};

static uint8_t command = 0;
static uint8_t delta = 0;
static uint32_t time_remain = 0;
static uint8_t buffer[256 * 8];
static struct {
    float temperature;
    uint32_t log_time;
} stack_temperature[STACK_SIZE_INTERN];
static uint16_t tail = 0;
static uint16_t head = 0;
static uint16_t delta_stack = 0;
static uint32_t deadline = 0;
static uint32_t temp_sampling_master = 200;

static inline uint8_t setting_slave(void); /* set slave before start modality master */
static inline void state_slave();
static inline void run_mode_slave(void);
static inline void run_mode_master(void);
static inline void push_stack(float temperature, uint32_t log_time);
static inline bool pull_stack(float *temperature, uint32_t *log_time);
static inline void sync_on_pc(void);

void MASTER_init() {
MX_GPIO_Init();
MX_USART1_UART_Init();
MX_USART2_UART_Init();
}

void MASTER_run() {
    if(init.save_setting) {
        command = setting_slave();
        HAL_UART_Transmit(&huart1, &command, sizeof(command), 100);
    }
    switch (init.samp_time) {
        case ZERO_S:
            run_mode_master();
            break;
        default:
            run_mode_slave();
            break;
    }
}

static inline uint8_t setting_slave(void) {
    uint8_t command = 0;
    init.save_setting = false;
    switch (init.temperature) {
        case KELVIN:
            CLEAR_BIT(command, 1 << 0);
            break;
        case CELSIUS:
            SET_BIT(command, 1 << 0);
            break;
        default:
            /* set kelvin */
            CLEAR_BIT(command, 1 << 0);
            break;
    }
    
    switch (init.log_time) {
        case LOG:
            SET_BIT(command, 1 << 1);
            break;
        case NO_LOG:
            CLEAR_BIT(command, 1 << 1);
            break;
        default:
            /* set no log */
            CLEAR_BIT(command, 1 << 1);
            break;
    }

    /* bit 3-5: sampling interval (0=0s, 1=10s, 2=30s, 3+=90s) */
    switch (init.samp_time) {
        case ZERO_S:
            CLEAR_BIT(command, 1 << 3);
            break;
        case TEN_S:
            SET_BIT(command, 1 << 3);
            break;
        case THIRTY_S:
            SET_BIT(command, 2 << 3);
            break;
        case NINETY_S:
            SET_BIT(command, 3 << 3);
            break;
        default:
            /* set zero */
            CLEAR_BIT(command, 1 << 3);
            break;
    }

    /* set no call*/
    CLEAR_BIT(command, 1 << 7);
    /* set save setting on slave */
    SET_BIT(command, 1 << 6);
    /* set slave mode: if samp 0 manual, else auto */
    switch (init.samp_time) {
        case ZERO_S:
            CLEAR_BIT(command, 1 << 2);
            break;
        default:
            SET_BIT(command, 1 << 2);
            break;
    }

    return command;
}

/* at each cycle read slave state and if exceed a percentage slave stack then sync and save on an internal array */
static inline void run_mode_slave(void) {
    state_slave();
    if(delta > (SIZE_STACK_SLAVE / 100.f) * init.percentage) {
        /* 0b10000000, bit 7 for call slave */
        uint8_t command_call = '\x80';
        HAL_UART_Transmit(&huart1, &command_call, 1, 100);
        HAL_UART_Receive(&huart1, buffer, (uint16_t)((((SIZE_STACK_SLAVE / 100.f) * init.percentage) + 1) * 8), 2000);

        for (uint16_t i = 0; i < ((SIZE_STACK_SLAVE / 100.f) * init.percentage) + 1; i++) {
            /* first save on temp variables to allow overwrite if stack_temperature[i] is not zero */
            uint32_t temp = 0;
            uint32_t l_time = 0;
            for (uint8_t j = 0; j < 4; j++) {
                temp |= ((uint32_t)buffer[i * 8 + j] << (j * 8));
                l_time |= ((uint32_t)buffer[i * 8 + j + 4] << (j * 8));
            }
            push_stack((float)(temp / 100.f), l_time);
        }
    }
    if(delta_stack == STACK_SIZE_INTERN - 1) sync_on_pc();
}

static inline void run_mode_master(void) {
    uint32_t tick_time = HAL_GetTick();
    if(tick_time > deadline) {
        deadline += temp_sampling_master;
        /* 0b10000000, bit 7 for call slave */
        uint8_t command_call = '\x80';
        HAL_UART_Transmit(&huart1, &command_call, sizeof(command_call), 100);
        HAL_UART_Receive(&huart1, buffer, 8, 100);
        uint32_t temp = 0;
        uint32_t l_time = 0;
        for (uint8_t j = 0; j < 4; j++) {
            temp |= ((uint32_t)buffer[j] << (j * 8));
            l_time |= ((uint32_t)buffer[j + 4] << (j * 8));
        }
        push_stack((float)(temp / 100.f), l_time);
    }
    if(delta_stack == STACK_SIZE_INTERN - 1) sync_on_pc();
}

static inline void state_slave(void) {
    #define SIZE_RECEIVE_STATUS 8
    const uint8_t command_state = '\x00';
    uint8_t receive_status[SIZE_RECEIVE_STATUS];
    HAL_UART_Transmit(&huart1, &command_state, sizeof(command_state), 100);
    HAL_UART_Receive(&huart1, receive_status, SIZE_RECEIVE_STATUS, 100);

    delta = (uint8_t)(receive_status[0] | (receive_status[1] << 8) | (receive_status[2] << 16) | (receive_status[3] << 24));
    time_remain = (receive_status[4] | (receive_status[5] << 8) | (receive_status[6] << 16) | (receive_status[7] << 24));
}

/* With limit of 512 character. If the stack is full, new values overwrite old ones. */
static inline void push_stack(float temperature, uint32_t log_time) {
    stack_temperature[head].temperature = temperature;
    stack_temperature[head].log_time = log_time;
    head++;
    head &= (STACK_SIZE_INTERN - 1);
    if(delta_stack < STACK_SIZE_INTERN) delta_stack++;
    else {
        tail++;
        tail &= (STACK_SIZE_INTERN - 1);
    }
}

static inline bool pull_stack(float *temperature, uint32_t *log_time) {
    if(delta_stack == 0) return false;
    *temperature = stack_temperature[tail].temperature;
    *log_time = stack_temperature[tail].log_time;
    tail++;
    tail &= (STACK_SIZE_INTERN - 1);
    delta_stack--;
    return true;
}

/* It doesn't set linker to support float on snprintf, so uses fmod */
static inline void sync_on_pc(void) {
    char buffer[20];
    float temperature;
    uint32_t log_time;
    while(pull_stack(&temperature,&log_time)) {
        uint8_t len = snprintf(buffer, sizeof(buffer), "%d.%d,%u\n", (int)temperature, (int)(fmod(temperature, 1.f) * 100), (unsigned int)log_time);
        HAL_UART_Transmit(&huart2, (const uint8_t *)buffer, len, 1000);
    }
}