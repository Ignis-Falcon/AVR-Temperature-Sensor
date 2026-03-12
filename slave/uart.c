#include "uart.h"

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL
#define FIFO_SIZE 64

struct {
  uint8_t data_array[FIFO_SIZE];
  volatile uint8_t head;
  volatile uint8_t tail;
} queque = {.head = 0, .tail = 0};

static inline void fifo_push(uint8_t data);
static inline bool fifo_pull(uint8_t *data);

void UART_init(uint16_t baud_rate) {
    uint16_t ubrr = F_CPU / 16 / baud_rate - 1;
    
    /* Set baud rate */
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;

    /* Set Control and Status Register B */
    UCSR0B = 1 << RXCIE0; /* RX Complete Interrupt Enable */
    UCSR0B |= 1 << RXEN0; /* Receiver Enable */
    UCSR0B |= 1 << TXEN0; /* Transmitter Enable */
    /* UCSZ02 is 0! */

    /* Set Control and Status Register C */
    UCSR0C =
      (1 << UCSZ01) | (1 << UCSZ00); /* 8-bit Character Size (UCSZ02 is 0)*/

    sei();
}

void UART_send_char(uint8_t ch) {
  /* wait until the last data in UDR0 is send */
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = ch;
}

void UART_send_string(char *string) {
  while (*string != '\0') {
    UART_send_char(*(string++));
  }
}

bool UART_receive_char(uint8_t *ch) {
  return fifo_pull(ch);
}

/* receive string with echo and backspace support for interactive CLI editing */
void UART_receive_interactive(uint8_t *string, uint8_t lenght) {
  uint8_t index = 0;
  uint8_t ch = '\0';
  while (1) {
    if (UART_receive_char(&ch)) {
      if (ch == '\b' || ch == 127) {
        if (index != 0) {
          index--;
          *(string + index) = '\0';
          UART_send_string("\b \b");
        }
      } else if (ch == '\r') {
        *(string + index) = '\0';
        UART_send_string("\r\n");
        return;
      } else if (index < lenght - 1) {
        *(string + index) = ch;
        index++;
        UART_send_char(ch);
      }
    }
  }
}

ISR(USART_RX_vect) {
  fifo_push(UDR0);
}

/* With limit of 64 character. If the stack is full, new values are discarded */
static inline void fifo_push(uint8_t data) {
  uint8_t next_head = (queque.head + 1) & (FIFO_SIZE - 1);
  if (next_head != queque.tail) {
    queque.data_array[queque.head] = data;
    queque.head = next_head;
  }
}

static inline bool fifo_pull(uint8_t *data) {
  if(queque.head == queque.tail) {
    return 0;
  }
  *data = queque.data_array[queque.tail];
  queque.tail = (queque.tail + 1) & (FIFO_SIZE - 1);
  return 1;
}

bool UART_available(void) {
  return (FIFO_SIZE + queque.head - queque.tail) & (FIFO_SIZE - 1);
}
