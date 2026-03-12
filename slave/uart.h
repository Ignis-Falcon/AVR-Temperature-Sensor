#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

void UART_init(uint16_t baud_rate);

void UART_send_char(uint8_t ch);
void UART_send_string(char * string);

void UART_receive_interactive(uint8_t *string, uint8_t lenght);
bool UART_receive_char(uint8_t *ch);

bool UART_available(void);

#endif