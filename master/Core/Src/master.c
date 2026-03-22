/* master.c - STM32 master firmware
 * Manages communication with AVR slave via UART1,
 * stores temperature samples in internal circular buffer,
 * and outputs data via UART2.
 */
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

static struct config {
    type_temp temperature; /* in celsius or kelvin */
    type_log log_time; /* time sampling in second by start slave mode */
    type_samp slave_sampling_time; /* in slave mode, time between two sampling */
    uint32_t master_sampling_time; /* as above but for master mode */
    bool save_setting; /* shows when necessity to save setting */
    uint8_t percentage_to_sync; /* in slave mode, percentage to synching when slave stack reach this % */
} init = {
    .temperature = CELSIUS,
    .log_time = LOG,
    .slave_sampling_time = NINETY_S,
    .master_sampling_time = 50000,
    .percentage_to_sync = 100,
    .save_setting = true
};

static uint8_t command_config = 0; /* command in 8bit to config slave */

/* state variables of slave stack */
static uint8_t delta_stack_slave = 0;
static uint32_t time_remain_stack_slave = 0;

static uint8_t buffer[SIZE_STACK_SLAVE * 8]; /* receive buffer, sized for max slave stack sync (256 samples * 8 bytes each) */

static struct circular_buffer {
    struct {
        float temperature;
        uint32_t log_time;
    } ring[STACK_SIZE_INTERN]; /* circular buffer to save temperature and log_time */
    uint16_t tail;
    uint16_t head;
    uint16_t delta;
} circular_buf = {
    .tail = 0,
    .head = 0,
    .delta = 0
};

static inline uint8_t setting_slave(void); /* set slave before start modality master */
static inline void state_slave();

static inline void run_mode_slave(void);
static inline void run_mode_master(void);

/* correlated to circular buffer */
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
        command_config = setting_slave();
        HAL_UART_Transmit(&huart1, &command_config, sizeof(command_config), 100);
    }
    switch (init.slave_sampling_time) {
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
    switch (init.slave_sampling_time) {
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
    switch (init.slave_sampling_time) {
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
    if(delta_stack_slave > (SIZE_STACK_SLAVE / 100.f) * init.percentage_to_sync) {
        /* 0b10000000, bit 7 for call slave */
        uint8_t command_call = '\x80';
        HAL_UART_Transmit(&huart1, &command_call, 1, 100);
        HAL_UART_Receive(&huart1, buffer, (uint16_t)((((SIZE_STACK_SLAVE / 100.f) * init.percentage_to_sync) + 1) * 8), 2000);

        for (uint16_t i = 0; i < ((SIZE_STACK_SLAVE / 100.f) * init.percentage_to_sync) + 1; i++) {
            /* first save on temp variables to allow overwrite if circular_buf.ring[i] is not zero */
            uint32_t temp = 0;
            uint32_t l_time = 0;
            for (uint8_t j = 0; j < 4; j++) {
                temp |= ((uint32_t)buffer[i * 8 + j] << (j * 8));
                l_time |= ((uint32_t)buffer[i * 8 + j + 4] << (j * 8));
            }
            push_stack((float)(temp / 100.f), l_time);
        }
    }
    if(circular_buf.delta == STACK_SIZE_INTERN - 1) sync_on_pc();
}

static inline void run_mode_master(void) {
    static uint32_t deadline;
    uint32_t tick_time = HAL_GetTick();
    if(tick_time > deadline) {
        deadline += init.master_sampling_time;
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
    if(circular_buf.delta == STACK_SIZE_INTERN - 1) sync_on_pc();
}

static inline void state_slave(void) {
    #define SIZE_RECEIVE_STATUS 8
    const uint8_t command_state = '\x00';
    uint8_t receive_status[SIZE_RECEIVE_STATUS];
    HAL_UART_Transmit(&huart1, &command_state, sizeof(command_state), 100);
    HAL_UART_Receive(&huart1, receive_status, SIZE_RECEIVE_STATUS, 100);

    delta_stack_slave = (uint8_t)(receive_status[0] | (receive_status[1] << 8) | (receive_status[2] << 16) | (receive_status[3] << 24));
    time_remain_stack_slave = (receive_status[4] | (receive_status[5] << 8) | (receive_status[6] << 16) | (receive_status[7] << 24));
}

/* With limit of 512 character. If the stack is full, new values overwrite old ones. */
static inline void push_stack(float temperature, uint32_t log_time) {
    circular_buf.ring[circular_buf.head].temperature = temperature;
    circular_buf.ring[circular_buf.head].log_time = log_time;
    circular_buf.head++;
    circular_buf.head &= (STACK_SIZE_INTERN - 1);
    if(circular_buf.delta < STACK_SIZE_INTERN) circular_buf.delta++;
    else {
        circular_buf.tail++;
        circular_buf.tail &= (STACK_SIZE_INTERN - 1);
    }
}

static inline bool pull_stack(float *temperature, uint32_t *log_time) {
    if(circular_buf.delta == 0) return false;
    *temperature = circular_buf.ring[circular_buf.tail].temperature;
    *log_time = circular_buf.ring[circular_buf.tail].log_time;
    circular_buf.tail++;
    circular_buf.tail &= (STACK_SIZE_INTERN - 1);
    circular_buf.delta--;
    return true;
}

/* float support not enabled in linker, temperature split in integer and decimal parts */
static inline void sync_on_pc(void) {
    char buffer[20];
    float temperature;
    uint32_t log_time;
    while(pull_stack(&temperature,&log_time)) {
        uint8_t len = snprintf(buffer, sizeof(buffer), "%d.%d,%u\n", (int)temperature, (int)(fmod(temperature, 1.f) * 100), (unsigned int)log_time);
        HAL_UART_Transmit(&huart2, (const uint8_t *)buffer, len, 1000);
    }
}