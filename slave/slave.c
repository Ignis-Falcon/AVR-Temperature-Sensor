#include "slave.h"
#include "uart.h"
#include "temp.h"
#include "timer.h"

#include <stdbool.h>
#include <stdint.h>

static bool sampling_mode = false;
static uint32_t sampling_interval = 0;
static uint32_t deadline = 0;
static uint32_t timer = 0;

void SLAVE_init(void) {
  TEMP_init();
  UART_init(9600);
}

void SLAVE_state_machine(void) {
    uint8_t ch_temp;
    uint32_t samp_temp;
    
    if(UART_available()) {
        UART_receive_char(&ch_temp);

        /* decode 8bit value for setting variable */
        /* bit 6: save configuration */
        if(ch_temp & (1 << 6)) {
            /* bit 0: temperature unit */
            TEMP_unit = ch_temp & (1 << 0) ? TEMP_CELSIUS : TEMP_KELVIN;
            /* bit 1: loggin state */
            TEMP_log_state = ch_temp & (1 << 1) ? TEMP_LOG : TEMP_NO_LOG;
            /* bit 2: simple mode */
            if(ch_temp & (1 << 2)) {
                sampling_mode = true;
                /* reset old data on change simple mode*/
                TEMP_reset_stack();
                TIMER_reset();
            } else {
                sampling_mode = false;
            }
            /* bit 3-5: sampling interval. 0 to 0s, 1 to 10s, 2 to 30s, 3 to 90s */
            switch ((ch_temp >> 3) & 0x7) {
                case 0x0:   sampling_interval = 0;
                            break;
                case 0x1:   sampling_interval = 10000;
                            break;
                case 0x2:   sampling_interval = 30000;
                            break;
                case 0x3:
                case 0x4:
                case 0x5:
                case 0x6:
                case 0x7:
                case 0x8:   sampling_interval = 90000;
                            break;
            }

            /* update deadline after setting */
            deadline = millis() + sampling_interval;
        }

        /* bit 7: request data to master*/
        if(ch_temp & 0x80) {
            if(sampling_mode) {
                TEMP_sync_send(UART_send_char);
            } else {
                samp_temp = TEMP_camp();
                TEMP_send_value(samp_temp, UART_send_char);
            }
        }

        /* all bit to 0: log stack status */
        if(ch_temp == 0x00) {
            TEMP_log_stack_status(sampling_interval, UART_send_char);
        }
    }

    /* sampling and update deadline */
    if(sampling_mode) {
        timer = millis();
        if(timer >= deadline) {
            deadline += sampling_interval;
            samp_temp = TEMP_camp();
            TEMP_push_stack(samp_temp);
        }
    }
}