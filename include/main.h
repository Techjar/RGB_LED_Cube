#include "stm32f4xx_conf.h"

//#define min(a,b) (a < b ? a : b)
//#define max(a,b) (a > b ? a : b)

#define STRIP_LENGTH 110
#define DATA_SIZE STRIP_LENGTH * 3
#define ZERO_BYTES 4
#define SEND_INTERVAL 1000000 / 180 // microseconds (change to 11 for 6-bit)
#define START_BYTE 0x55

#define RX_BUFFER_SIZE 4096
#define DMA_BUFFER_SIZE 128

//uint8_t spi_buffer[25]; // 25 8-bit shift registers
uint8_t dma_buffer[DMA_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile int rx_buffer_head;
volatile int rx_buffer_tail;

void Send_Frame(void);
void SPI_Transfer(uint8_t value);
void Serial_Write(uint8_t value);
void Serial_WriteBytes(uint8_t* buf, int length);
//uint8_t Serial_Read();
void Serial_ReadBytes(uint8_t* buf, int length);
int Serial_Available();
