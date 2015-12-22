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

uint8_t* front_buffer;
uint8_t* back_buffer;
uint8_t buffer_ready;

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

	//RCC_PCLK2Config(RCC_HCLK_Div16);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM2, ENABLE);
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

	USART_InitStruct.USART_BaudRate = 500000;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStruct);

	// SPI pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	SPI_I2S_DeInit(SPI1);
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // 84 MHz bus speed
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_CRCPolynomial = 0;
	SPI_Init(SPI1, &SPI_InitStruct);

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

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	USART_Cmd(USART2, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	SPI_Cmd(SPI1, ENABLE);

	TIM_TimeBaseInitStruct.TIM_Period = SEND_INTERVAL - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 84 - 1; // 1 Mhz
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	TIM_Cmd(TIM2, ENABLE);

	front_buffer = (uint8_t*)malloc(DATA_SIZE);
	back_buffer = (uint8_t*)malloc(DATA_SIZE);
	memset(front_buffer, 0b10000000, DATA_SIZE);
	Send_Frame();

	while (1) {
		if (!buffer_ready && Serial_Available() >= DATA_SIZE) {
			Serial_ReadBytes(back_buffer, DATA_SIZE);
			buffer_ready = 1;
		}
		if (buffer_ready) {
			buffer_swap(&front_buffer, &back_buffer);
			buffer_ready = 0;
		}
		if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update)) {
			TIM_ClearFlag(TIM2, TIM_FLAG_Update);
			Send_Frame();
		}
	}

	return 0;
}

void Send_Frame(void) {
	for (int i = 0; i < DATA_SIZE; i++) SPI_Transfer(front_buffer[i]);
	for (int i = 0; i < ZERO_BYTES; i++) SPI_Transfer(0);
}

void SPI_Transfer(uint8_t value) {
	SPI_I2S_SendData(SPI1, value);
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
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
	int total = 0;
	while (total < length) {
		int amount = rx_buffer_tail + (length - total) >= RX_BUFFER_SIZE ? RX_BUFFER_SIZE - rx_buffer_tail : length - total;
		memcpy(&buf[total], &rx_buffer[rx_buffer_tail], amount);
		rx_buffer_tail = (rx_buffer_tail + amount) % RX_BUFFER_SIZE;
		total += amount;
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

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
