#include "timer.h"

#include <avr/interrupt.h>
#include <avr/io.h>

volatile uint32_t _millis = 0;

void TIMER_init(void) {
    /* set mode ctc */
    TCCR0A = 1 << WGM01;

    /* compare register A to call interrupt */
    OCR0A = 249;

    /* interrupt enable compare register A */
    TIMSK0 = 1 << OCIE0A;

    /* 1024 prescaler */
    TCCR0B |= (1 << CS01) | (1 << CS00);

    sei();
}

uint32_t millis(void) {
    cli();
    uint32_t temp = _millis;
    sei();
    return temp;
} 


ISR(TIMER0_COMPA_vect) {
    _millis++;
}

void TIMER_reset(void) {
    cli();
    _millis = 0;
    sei();
}