#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

void SysTick_Configuration(void);
void delay(uint32_t time);
//void delayMicros(uint32_t time);
uint32_t timeMillis();
//uint32_t timeMicros();

#endif
