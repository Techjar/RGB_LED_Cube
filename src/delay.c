#include "stm32f4xx.h"

static volatile uint32_t timing_delay;
static volatile uint32_t time_counter;

void SysTick_Configuration(void) {
	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1);
	}
}

void delay(uint32_t time) {
	timing_delay = time;
	while (timing_delay != 0);
}

/*void delayMicros(uint32_t time) {
	timing_delay = time;
	while (timing_delay != 0);
}*/

uint32_t timeMillis() {
	return time_counter;
}

/*uint32_t timeMicros() {
	return time_counter;
}*/

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) {
	time_counter++;
	if (timing_delay != 0) {
		timing_delay--;
	}
}
