#include "stm32f4xx_sn8200.h"
#include "stm32f4xx_conf.h"
#include "delay.h"

void SN8200_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;

	for (int i = 0; i < SN8200_DMA_TX_BUFFER; i++) empty_buffer[i] = 0;
	for (int i = 0; i < SN8200_DMA_RX_BUFFER; i++) rx_buffer[i] = 0;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 /*| GPIO_Pin_12*/;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->BSRRL = GPIO_Pin_12; // set NSS high
	*/

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);

	SPI_I2S_DeInit(SN8200_SPI);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 0;
	SPI_Init(SN8200_SPI, &SPI_InitStruct);

	DMA_DeInit(SN8200_DMA_RX_STREAM);
	while (DMA_GetCmdStatus(SN8200_DMA_RX_STREAM) == ENABLE);
	DMA_InitStruct.DMA_Channel = SN8200_DMA_RX_CHANNEL;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(SN8200_SPI->DR);
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&rx_buffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = SN8200_DMA_RX_BUFFER;
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
	DMA_Init(SN8200_DMA_RX_STREAM, &DMA_InitStruct);

	DMA_DeInit(SN8200_DMA_TX_STREAM);
	while (DMA_GetCmdStatus(SN8200_DMA_TX_STREAM) == ENABLE);
	DMA_InitStruct.DMA_Channel = SN8200_DMA_TX_CHANNEL;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(SN8200_SPI->DR);
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&empty_buffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = SN8200_DMA_TX_BUFFER;
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
	DMA_Init(SN8200_DMA_TX_STREAM, &DMA_InitStruct);

	DMA_ITConfig(SN8200_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
	//DMA_ITConfig(SN8200_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
	SPI_I2S_DMACmd(SN8200_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SN8200_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

	//SPI_I2S_ITConfig(SN8200_SPI, SPI_I2S_IT_RXNE, ENABLE);

	// RX
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	// TX
	/*NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);*/

	/*SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);
	EXTI_InitStruct.EXTI_Line = EXTI_Line1;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);*/

	SPI_Cmd(SN8200_SPI, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	delay(500);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	delay(7000);

	DMA_Cmd(SN8200_DMA_RX_STREAM, ENABLE);
	//GPIOB->BSRRH = GPIO_Pin_12; // set NSS low
	SN8200_StartTransfer();
}

void SN8200_SendPacket(uint8_t ack_required, uint8_t command, uint8_t* payload, uint16_t length) {
	assert_param(length <= SN8200_MAX_PAYLOAD);

	GPIOD->BSRRL = GPIO_Pin_14;
	SN8200_StopTransfer();
	uint8_t l0 = (length & 0b1111111);
	uint8_t l1 = ((length >> 7) & 0b111111) | (ack_required << 6);
	SN8200_SendRaw(SN8200_SOM);
	SN8200_SendRaw(l0 | 0b10000000);
	SN8200_SendRaw(l1 | 0b10000000);
	SN8200_SendRaw(command | 0b10000000);
	for (int i = 0; i < length; i++) SN8200_SendRaw(payload[i]);
	uint8_t checksum = l0 | 0b10000000;
	checksum += l1 | 0b10000000;
	checksum += command | 0b10000000;
	SN8200_SendRaw(checksum | 0b10000000);
	SN8200_SendRaw(SN8200_EOM);
	SN8200_StartTransfer();
	GPIOD->BSRRH = GPIO_Pin_14;
}

/*void SN8200_BufferByte(uint8_t value) {
	write_waiting = 1;
	while(transfer_active);
	if (tx_buffered_amount >= SN8200_DMA_TX_BUFFER) {
		SN8200_StartTransfer();
		while(transfer_active);
	}
	tx_buffer[tx_buffered_amount++] = value;
	write_waiting = 0;
}*/

void SN8200_SendRaw(uint8_t value) {
	tx_buffer[tx_buffered_amount++] = value;
	if (value == SN8200_EOM || tx_buffered_amount >= SN8200_DMA_TX_BUFFER) {
		SN8200_SendBuffer(tx_buffer, tx_buffered_amount);
		tx_buffered_amount = 0;
	}
}

void SN8200_SendBuffer(uint8_t* buf, uint32_t len) {
	for (int i = 0; i < len; i++) {
		SPI_I2S_SendData(SN8200_SPI, buf[i]);
		while (!SPI_I2S_GetFlagStatus(SN8200_SPI, SPI_I2S_FLAG_TXE));
	}
}

void SN8200_StartTransfer(void) {
	//SN8200_DMA_TX_STREAM->NDTR = tx_buffered_amount;
	//GPIOB->BSRRH = GPIO_Pin_12;
	//SPI_Cmd(SN8200_SPI, ENABLE);
	//DMA_Cmd(SN8200_DMA_RX_STREAM, ENABLE);
	DMA_Cmd(SN8200_DMA_TX_STREAM, ENABLE);
	while (DMA_GetCmdStatus(SN8200_DMA_TX_STREAM) != ENABLE);
	transfer_active = 1;
}

void SN8200_StopTransfer(void) {
	transfer_active = 0;
	DMA_Cmd(SN8200_DMA_TX_STREAM, DISABLE);
	while (DMA_GetCmdStatus(SN8200_DMA_TX_STREAM) == ENABLE);
	//DMA_Cmd(SN8200_DMA_RX_STREAM, DISABLE);
	//while (DMA_GetCmdStatus(SN8200_DMA_RX_STREAM) == ENABLE);
	//SPI_Cmd(SN8200_SPI, DISABLE);
	//GPIOB->BSRRL = GPIO_Pin_12;
	//tx_buffered_amount = 0;
}

void SN8200_RecvPacket(uint8_t command, uint8_t* payload, uint16_t length) {
	// TODO
	//GPIOD->BSRRL = GPIO_Pin_12;
	if (length > 0 && payload[0] == SN8200_WIFI_GET_STATUS_RSP) GPIOD->ODR ^= GPIO_Pin_15;
}

// RX
void DMA1_Stream3_IRQHandler(void) {
	if (DMA_GetITStatus(SN8200_DMA_RX_STREAM, SN8200_DMA_RX_IT_TCIF)) {
		GPIOD->BSRRL = GPIO_Pin_13;
		/*volatile uint8_t* buffer;
		if (active_buffer == 0) {
			SN8200_DMA_RX_STREAM->M0AR = (uint32_t)&rx_buffer1;
			buffer = rx_buffer0;
			active_buffer = 1;
		} else if (active_buffer == 1) {
			SN8200_DMA_RX_STREAM->M0AR = (uint32_t)&rx_buffer0;
			buffer = rx_buffer1;
			active_buffer = 0;
		}*/
		for (int i = 0; i < SN8200_DMA_RX_BUFFER; i++) {
			if (pending_read_index < 4 || pending_read_index - 4 < pending_frame.length) {
				switch (pending_read_index) {
					case 0:
						if (rx_buffer[i] == SN8200_SOM) {
							//GPIOD->BSRRL = GPIO_Pin_12;
							GPIOD->ODR ^= GPIO_Pin_12;
							pending_read_index++;
						}
						break;
					case 1:
						pending_frame.l0 = rx_buffer[i] & 0b1111111;
						pending_read_index++;
						break;
					case 2:
						pending_frame.l1 = rx_buffer[i] & 0b111111;
						pending_frame.length = pending_frame.l0 | (pending_frame.l1 << 7);
						pending_frame.ack = (rx_buffer[i] & 0b1000000) != 0;
						pending_frame.payload = (uint8_t*)malloc(pending_frame.length);
						pending_read_index++;
						break;
					case 3:
						pending_frame.command = rx_buffer[i] & 0b1111111;
						pending_read_index++;
						break;
					default:
						pending_frame.payload[pending_read_index - 4] = rx_buffer[i];
						pending_read_index++;
						break;
				}
			} else {
				switch (pending_read_index - pending_frame.length - 4) {
					case 0:
						pending_frame.checksum = rx_buffer[i] & 0b1111111;
						pending_read_index++;
						break;
					case 1:
						if (rx_buffer[i] == SN8200_EOM) {
							//GPIOD->BSRRH = GPIO_Pin_12;
							uint8_t checksum = pending_frame.l0 | 0b10000000;
							checksum += pending_frame.l1 | (pending_frame.ack << 6) | 0b10000000;
							checksum += pending_frame.command | 0b10000000;
							checksum &= 0b1111111;
							if (checksum == pending_frame.checksum) {
								if (pending_frame.ack) {
									SN8200_SendPacket(0, SN8200_CMD_ACK, NULL, 0);
								}
								SN8200_RecvPacket(pending_frame.command, pending_frame.payload, pending_frame.length);
							} else if (pending_frame.ack) {
								SN8200_SendPacket(0, SN8200_CMD_NAK, NULL, 0);
							}
							free(pending_frame.payload);
							pending_read_index = 0;
						}
						break;
				}
			}
		}
		GPIOD->BSRRH = GPIO_Pin_13;
		DMA_ClearITPendingBit(SN8200_DMA_RX_STREAM, SN8200_DMA_RX_IT_TCIF);
		DMA_Cmd(SN8200_DMA_RX_STREAM, ENABLE);
	}
}

// RX
/*void SPI2_IRQHandler(void) {
	if (SPI_I2S_GetITStatus(SN8200_SPI, SPI_I2S_IT_RXNE)) {
		if (SPI_I2S_ReceiveData(SN8200_SPI) == SN8200_SOM) GPIOD->BSRRL = GPIO_Pin_12;
		SPI_I2S_ClearITPendingBit(SN8200_SPI, SPI_I2S_IT_RXNE);
	}
}*/

// TX
void DMA1_Stream4_IRQHandler(void) {
	if (DMA_GetITStatus(SN8200_DMA_TX_STREAM, SN8200_DMA_TX_IT_TCIF)) {
		/*GPIOD->BSRRL = GPIO_Pin_13;
		if (receive_active && !write_waiting) {
			tx_buffered_amount = SN8200_DMA_TX_BUFFER;
			SN8200_DMA_TX_STREAM->M0AR = (uint32_t)&empty_buffer;
			SN8200_StartTransfer();
		} else {
			SN8200_DMA_TX_STREAM->M0AR = (uint32_t)&tx_buffer;
			SN8200_StartTransfer();
		}*/
		DMA_ClearITPendingBit(SN8200_DMA_TX_STREAM, SN8200_DMA_TX_IT_TCIF);
	}
}

void EXTI1_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line1)) {
		receive_active = ((GPIOA->IDR & GPIO_Pin_1) != 0);
		if (receive_active) GPIOD->BSRRL = GPIO_Pin_15;
		else GPIOD->BSRRH = GPIO_Pin_15;
		/*if (tx_buffered_amount == 0 && !write_waiting) {
			if (receive_active) {
				SN8200_StartTransfer();
			} else {
				//SN8200_StopTransfer();
			}
		}*/
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
