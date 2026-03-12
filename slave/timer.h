#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

void TIMER_reset(void);
void TIMER_init(void);
uint32_t millis(void);

#endif