#include "adc.h"

#include <avr/io.h>



void ADC_init(void) {
    
    /* AVcc */
    ADMUX |= 1 << REFS0;

    /* setting prescaler (max - 128)*/
    ADCSRA |= 0x7;

    /* enable ADC */
    ADCSRA |= 1 << ADEN;
}

uint16_t ADC_read(void) {
    /* default channel 0 selected */

    /* start conversion */
    ADCSRA |= 1 << ADSC;

    /* ADSC return to 0 when conversion is completed */
    while(ADCSRA & (1 << ADSC));

    return ADCL | (ADCH << 0x8);
}