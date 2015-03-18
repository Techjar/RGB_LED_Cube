//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include "stm32f4xx_conf.h"
//#include "stm32f4xx_sn8200.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "delay.h"
#include "main.h"

// ----------------------------------------------------------------------------
//
// STM32F4 empty sample (trace via ITM).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

volatile uint8_t* front_buffer;
volatile uint8_t* back_buffer;
volatile uint8_t buffer_ready = 0;
volatile int level = 0; // this increments through the anode levels

int min(int a, int b) {
	return a < b ? a : b;
}

int max(int a, int b) {
	return a > b ? a : b;
}

void buffer_swap(uint8_t** a, uint8_t** b) {
	uint8_t* temp = *a;
	*a = *b;
	*b = temp;
}

int main(void) {
	SysTick_Configuration();
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_OCStructInit(&TIM_OCInitStruct);

	//RCC_PCLK2Config(RCC_HCLK_Div16);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	// LED pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	// USART pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	// SPI pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);

	// Timer 3 & 4 pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

	USART_InitStruct.USART_BaudRate = 2000000;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStruct);

	SPI_I2S_DeInit(SPI1);
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_CRCPolynomial = 0;
	SPI_Init(SPI1, &SPI_InitStruct);

	// LATCH, DCPRG, VPRG & 74HC595 BLANK pins
	GPIO_InitStruct.GPIO_Pin = GPIO_LATCH_PIN | GPIO_DCPRG_PIN | GPIO_VPRG_PIN | GPIO_74HC595OE_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIO_PORT, &GPIO_InitStruct);
	GPIO_PORT->BSRRL = GPIO_74HC595OE_PIN;
	GPIO_PORT->BSRRL = GPIO_DCPRG_PIN;

	// UART DMA
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&dma_buffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = DMA_BUFFER_SIZE;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStruct);

	// SPI DMA
	DMA_InitStruct.DMA_Channel = DMA_Channel_3;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(SPI1->DR);
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&spi_buffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = SPI_BUFFER_SIZE;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStruct);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	/*DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);*/

	USART_Cmd(USART2, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	SPI_Cmd(SPI1, ENABLE);

	TIM_TimeBaseInitStruct.TIM_Period = 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = ((TIM_APB1_FREQ / TLC5940_GSCLK_FREQ) / 4) - 1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_TimeBaseInitStruct.TIM_Period = TLC5940_GSCLK_COUNTS + TLC5940_BLANK_COUNT;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = TLC5940_BLANK_COUNT;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM4, TIM_TS_ITR2);
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_External1);
	TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);

	front_buffer = (uint8_t*)malloc(DATA_SIZE);
	back_buffer = (uint8_t*)malloc(DATA_SIZE);
	memset(front_buffer, 0, DATA_SIZE);

	{ // Fill the dot correction register and zero the anode control register
		GPIO_PORT->BSRRL = GPIO_VPRG_PIN;
		delay(1);
		int size = (TLC5940_DATA_SIZE / 2) + 1;
		DMA2_Stream5->NDTR = size;
		memset(spi_buffer, 0xFF, size - 1);
		spi_buffer[size] = 0;
		DMA_Cmd(DMA2_Stream5, ENABLE);
		while (DMA_GetCmdStatus(DMA2_Stream5));
		DMA2_Stream5->NDTR = SPI_BUFFER_SIZE;
		delay(1);
		GPIO_PORT->BSRRL = GPIO_LATCH_PIN;
		delay(1);
		GPIO_PORT->BSRRH = GPIO_LATCH_PIN;
		delay(1);
		GPIO_PORT->BSRRH = GPIO_VPRG_PIN;
		delay(1);
		GPIO_PORT->BSRRH = GPIO_74HC595OE_PIN;
		delay(1);
	}

	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	while (1) {
		if (!buffer_ready && Serial_Available() >= DATA_SIZE) {
			Serial_ReadBytes(back_buffer, DATA_SIZE);
			buffer_ready = 1;
		}
	}

	return 0;
}

void Serial_Write(uint8_t value) {
	USART_SendData(USART2, value);
	while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
}

void Serial_WriteBytes(uint8_t* buf, int length) {
	for (int i = 0; i < length; i++) {
		USART_SendData(USART2, buf[i]);
		while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
	}
}

void Serial_ReadBytes(uint8_t* buf, int length) {
	while (Serial_Available() < length);
	int amount = rx_buffer_tail + length >= RX_BUFFER_SIZE ? RX_BUFFER_SIZE - rx_buffer_tail : length;
	memcpy(&buf[0], &rx_buffer[rx_buffer_tail], amount);
	rx_buffer_tail = (rx_buffer_tail + amount) % RX_BUFFER_SIZE;
	if (amount < length) {
		memcpy(&buf[amount], &rx_buffer[rx_buffer_tail], length - amount);
		rx_buffer_tail = (rx_buffer_tail + (length - amount)) % RX_BUFFER_SIZE;
	}
}

int Serial_Available() {
	return (RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail) % RX_BUFFER_SIZE;
}

void DMA1_Stream5_IRQHandler(void) {
	if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5)) {
		int amount = rx_buffer_head + DMA_BUFFER_SIZE >= RX_BUFFER_SIZE ? RX_BUFFER_SIZE - rx_buffer_head : DMA_BUFFER_SIZE;
		memcpy(&rx_buffer[rx_buffer_head], &dma_buffer[0], amount);
		rx_buffer_head = (rx_buffer_head + amount) % RX_BUFFER_SIZE;
		if (amount < DMA_BUFFER_SIZE) {
			memcpy(&rx_buffer[rx_buffer_head], &dma_buffer[amount], DMA_BUFFER_SIZE - amount);
			rx_buffer_head = (rx_buffer_head + (DMA_BUFFER_SIZE - amount)) % RX_BUFFER_SIZE;
		}
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
	}
}

void TIM4_IRQHandler(void) {
	if (TIM_GetFlagStatus(TIM4,TIM_IT_CC1)) {
		if (TIM4->CR1 & TIM_CR1_DIR) {
			if (buffer_ready && level == 0) {
				buffer_swap(&front_buffer, &back_buffer);
				buffer_ready = 0;
			}
			while (DMA_GetCmdStatus(DMA2_Stream5));
			GPIO_PORT->BSRRL = GPIO_LATCH_PIN;
			memcpy(&spi_buffer[0], &front_buffer[TLC5940_DATA_SIZE * level], TLC5940_DATA_SIZE);
			spi_buffer[TLC5940_DATA_SIZE] = 1 << level;
			if (level++ == 8) level = 0;
			GPIO_PORT->BSRRH = GPIO_LATCH_PIN;
			DMA_Cmd(DMA2_Stream5, ENABLE);
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
