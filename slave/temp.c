#include "temp.h"
#include "adc.h"
#include "timer.h"

#include <math.h>
#include <stdint.h>

#define RESISTENCE  98100.f
#define RES_NTC     100000.f
#define PRESCALER   1023.f
#define BETA        3950.f
#define T0          (273.15 + 25)

#define STACK_SIZE 256

static uint32_t stack_memory[STACK_SIZE];
static uint16_t head = 0;
static uint16_t tail = 0;
uint16_t delta = 0;

TEMP_unit_temp TEMP_unit = TEMP_KELVIN;
TEMP_state_log TEMP_log_state = TEMP_NO_LOG;

static float TEMP_read(void);
static float TEMP_pull_temperature(uint32_t camp);
static float TEMP_pull_log(uint32_t camp);
static uint32_t TEMP_pull_stack(void);
static inline void TEMP_send_32bit_value(int32_t value, void (*func_transmit)(unsigned char ch));

void TEMP_init(void) {
    ADC_init();
    TIMER_init();
}

float TEMP_read(void) {
    uint16_t analog_value;
    analog_value = ADC_read();

    /* Steinhart–Hart equation with only beta parameter */
    float ln = log((RESISTENCE * ((PRESCALER / analog_value) - 1)) / RES_NTC);
    float bb = (1.0 / BETA) * ln;
    float tt = (1.0 / T0) + bb;
    tt = 1.0 / tt;

    return tt;
}

/* Capture a temperature reading and of time, then  this value are packed on a 32-bit variable.*/
uint32_t TEMP_camp(void) {

    float temp;
    uint32_t camp;

    temp = TEMP_read();

    /* add offset if kelvin */
    if(TEMP_unit == TEMP_KELVIN) {
        TEMP_unit = TEMP_KELVIN;
        temp -= 223.15;
    }

    /* conversion if celsius and add offset*/
    if(TEMP_unit == TEMP_CELSIUS) {
        TEMP_unit = TEMP_CELSIUS;
        temp -= 273.15;
        temp += 50;
    }

    /* hight bits for temperature */
    camp = ((uint32_t)(temp * 100) & 0x1fff) << 17;

    /* low bits for log */
    if(TEMP_log_state == TEMP_LOG) {
        uint32_t log = millis() / 1000;
        camp |= (log & 0x1FFFF);
    }

    return camp;
}

float TEMP_pull_temperature(uint32_t camp) {

    float conversion = (camp >> 17) / 100.f;

    /* add offset if kelvin */
    if(TEMP_unit == TEMP_KELVIN) {
        conversion += 223.15;
    }

    /* conversion if celsius and add offset*/
    if(TEMP_unit == TEMP_CELSIUS) {
        conversion -= 50;
    }

    return conversion;
}

float TEMP_pull_log(uint32_t camp) {
    return (float)(camp & 0x1FFFF);
}

/* With limit of 256 character. If the stack is full, new values overwrite old ones. */
void TEMP_push_stack(uint32_t value) {
    stack_memory[head] = value;
    head++;
    head &= (STACK_SIZE - 1);
    if(delta < STACK_SIZE) delta++;
    else {
        tail++;
        tail &= (STACK_SIZE - 1);
    }
}

uint32_t TEMP_pull_stack(void) {
    if(delta == 0) return 0;
    uint32_t temp;
    temp = stack_memory[tail];
    tail++;
    tail &= (STACK_SIZE - 1);
    delta--;
    return temp;
}

/* Send a 32-bit value through the specified transmit function */
static inline void TEMP_send_32bit_value(int32_t value, void (*func_transmit)(unsigned char ch)) {
    uint8_t ch;
    ch = value & 0xff;
    func_transmit(ch);
    ch = (value >> 8) & 0xff;
    func_transmit(ch);
    ch = (value >> 16) & 0xff;
    func_transmit(ch);
    ch = (value >> 24) & 0xff;
    func_transmit(ch);
}

/* extract by 32bit value temperature and log time, in a 32bit variable each one. Then use a callback to send data through a protocol-specific send function. */
void TEMP_send_value(uint32_t value, void (*func_transmit)(unsigned char ch)) {
    int32_t tempe = (int32_t)(TEMP_pull_temperature(value) * 100);
    uint32_t logge = (uint32_t)TEMP_pull_log(value);
    TEMP_send_32bit_value(tempe, func_transmit);
    TEMP_send_32bit_value((int32_t)logge, func_transmit);
}

void TEMP_sync_send(void (*func_transmit)(unsigned char ch)) {
    uint32_t value;
    while(delta != 0) {
    value = TEMP_pull_stack();
    TEMP_send_value(value, func_transmit);
    }
}

void TEMP_reset_stack(void) {
    head = tail = delta = 0;
}

/* Log the stack status: delta and total sampling time */
void TEMP_log_stack_status(uint32_t sampling_interval, void (*func_transmit)(unsigned char ch)) {
    uint32_t tot_time_sampling = (STACK_SIZE - delta) * (sampling_interval / 1000);
    TEMP_send_32bit_value((int32_t)delta, func_transmit);
    TEMP_send_32bit_value((int32_t)tot_time_sampling, func_transmit);

}