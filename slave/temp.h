#ifndef TEMP_H
#define TEMP_H

#include <stdint.h>

extern uint16_t delta;

typedef enum {TEMP_KELVIN, TEMP_CELSIUS} TEMP_unit_temp;
typedef enum {TEMP_NO_LOG, TEMP_LOG} TEMP_state_log;

extern TEMP_unit_temp TEMP_unit;
extern TEMP_state_log TEMP_log_state;

void TEMP_init(void);

uint32_t TEMP_camp(void);
void TEMP_push_stack(uint32_t value);
void TEMP_send_value(uint32_t value, void (*func_transmit)(unsigned char ch));
void TEMP_sync_send(void (*func_transmit)(unsigned char ch));
void TEMP_reset_stack(void);
void TEMP_log_stack_status(uint32_t sampling_interval, void (*func_transmit)(unsigned char ch));

#endif