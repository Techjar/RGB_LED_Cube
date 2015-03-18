#include "stm32f4xx_conf.h"

//#define min(a,b) (a < b ? a : b)
//#define max(a,b) (a > b ? a : b)

#define DATA_SIZE 2304
#define TLC5940_DATA_SIZE 288

#define RX_BUFFER_SIZE 8192
#define DMA_BUFFER_SIZE 192
#define SPI_BUFFER_SIZE TLC5940_DATA_SIZE + 1

#define TLC5940_GSCLK_COUNTS 4096
#define TLC5940_GSCLK_FREQ 5000000
#define TLC5940_BLANK_COUNT 250
#define TIM_APB1_FREQ 84000000

#define GPIO_PORT GPIOE
#define GPIO_LATCH_PIN GPIO_Pin_7
#define GPIO_DCPRG_PIN GPIO_Pin_8
#define GPIO_VPRG_PIN GPIO_Pin_9
#define GPIO_74HC595OE_PIN GPIO_Pin_10

uint8_t dma_buffer[DMA_BUFFER_SIZE];
uint8_t spi_buffer[SPI_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile int rx_buffer_head;
volatile int rx_buffer_tail;

void Serial_Write(uint8_t value);
void Serial_WriteBytes(uint8_t* buf, int length);
//uint8_t Serial_Read();
void Serial_ReadBytes(uint8_t* buf, int length);
int Serial_Available();
