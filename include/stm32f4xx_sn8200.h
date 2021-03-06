#include "stm32f4xx_conf.h"
#include "stdlib.h"

#define SN8200_MAX_PAYLOAD 8000

#define SN8200_SOM (uint8_t)0x02
#define SN8200_EOM (uint8_t)0x04
#define SN8200_CMD_NAK (uint8_t)0x00
#define SN8200_CMD_ACK (uint8_t)0x7F
#define SN8200_CMD_GEN (uint8_t)0x01
#define SN8200_CMD_SNIC (uint8_t)0x70
#define SN8200_CMD_WIFI (uint8_t)0x50

#define SN8200_ACK_REQUIRED (uint8_t)1
#define SN8200_ACK_NOT_REQUIRED (uint8_t)0

#define IPADDR_ANY 0x00000000
#define IPADDR_NONE 0xFFFFFFFF

#define SN8200_DMA_RX_STREAM DMA1_Stream3
#define SN8200_DMA_TX_STREAM DMA1_Stream4
#define SN8200_DMA_RX_CHANNEL DMA_Channel_0
#define SN8200_DMA_TX_CHANNEL DMA_Channel_0
#define SN8200_DMA_RX_FLAG_TCIF DMA_FLAG_TCIF3
#define SN8200_DMA_TX_FLAG_TCIF DMA_FLAG_TCIF4
#define SN8200_DMA_RX_IT_TCIF DMA_IT_TCIF3
#define SN8200_DMA_TX_IT_TCIF DMA_IT_TCIF4
#define SN8200_DMA_RX_BUFFER (uint16_t)2048
#define SN8200_DMA_TX_BUFFER (uint16_t)2048
#define SN8200_SPI SPI2
#define SN8200_GPIO_PORT GPIOA
#define SN8200_GPIO_PIN 1

typedef enum {
	SN8200_GEN_FW_VER_GET_REQ = 0x08,
	SN8200_GEN_RESTORE_REQ,
	SN8200_GEN_FW_VER_GET_RSP = 0x88,
	SN8200_GEN_RESTORE_RSP,
} SN8200_GEN;

typedef enum {
	SN8200_SNIC_INIT_REQ = 0,
	SN8200_SNIC_CLEANUP_REQ,
	SN8200_SNIC_SEND_FROM_SOCKET_REQ,
	SN8200_SNIC_CLOSE_SOCKET_REQ,
	SN8200_SNIC_SOCKET_PARTIAL_CLOSE_REQ,
	SN8200_SNIC_GETSOCKOPT_REQ,
	SN8200_SNIC_SETSOCKOPT_REQ,
	SN8200_SNIC_SOCKET_GETNAME_REQ,
	SN8200_SNIC_SEND_ARP_REQ,
	SN8200_SNIC_GET_DHCP_INFO_REQ,
	SN8200_SNIC_RESOLVE_NAME_REQ,
	SN8200_SNIC_IP_CONFIG_REQ,

	SN8200_SNIC_TCP_CREATE_SOCKET_REQ = 0x10,
	SN8200_SNIC_TCP_CREATE_CONNECTION_REQ,
	SN8200_SNIC_TCP_CONNECT_TO_SERVER_REQ,

	SN8200_SNIC_UDP_CREATE_SOCKET_REQ,
	SN8200_SNIC_UDP_START_RECV_REQ,
	SN8200_SNIC_UDP_SIMPLE_SEND_REQ,
	SN8200_SNIC_UDP_SEND_FROM_SOCKET_REQ,

	SN8200_SNIC_HTTP_REQ,
	SN8200_SNIC_HTTP_MORE_REQ,
	SN8200_SNIC_HTTPS_REQ,

	SN8200_SNIC_TCP_CREATE_ADV_TLS_SOCKET_REQ,
	SN8200_SNIC_TCP_CREATE_SIMPLE_TLS_SOCKET_REQ,

	SN8200_SNIC_TCP_CONNECTION_STATUS_IND = 0x20,
	SN8200_SNIC_TCP_CLIENT_SOCKET_IND,
	SN8200_SNIC_CONNECTION_RECV_IND,
	SN8200_SNIC_UDP_RECV_IND,
	SN8200_SNIC_ARP_REPLY_IND,
	SN8200_SNIC_HTTP_RSP_IND,

	SN8200_SNIC_SEND_RSP = 0x82,
	SN8200_SNIC_CLOSE_SOCKET_RSP,
	SN8200_SNIC_GET_DHCP_INFO_RSP = 0x89,
	SN8200_SNIC_IP_CONFIG_RSP = 0x8B,
	SN8200_SNIC_TCP_CREATE_SOCKET_RSP = 0x90,
	SN8200_SNIC_TCP_CREATE_CONNECTION_RSP,
	SN8200_SNIC_TCP_CONNECT_TO_SERVER_RSP,

	SN8200_SNIC_UDP_CREATE_SOCKET_RSP = 0x93,
	SN8200_SNIC_UDP_SEND_FROM_SOCKET_RSP = 0x96,

	SN8200_SNIC_TCP_CREATE_ADV_TLS_SOCKET_RSP = 0x9A,
	SN8200_SNIC_TCP_CREATE_SIMPLE_TLS_SOCKET_RSP,
} SN8200_SNIC;

typedef enum {
	SN8200_SNIC_SUCCESS = 0,
	SN8200_SNIC_FAIL,
	SN8200_SNIC_INIT_FAIL,
	SN8200_SNIC_CLEANUP_FAIL,
	SN8200_SNIC_GETADDRINFO_FAIL,
	SN8200_SNIC_CREATE_SOCKET_FAIL,
	SN8200_SNIC_BIND_SOCKET_FAIL,
	SN8200_SNIC_LISTEN_SOCKET_FAIL,
	SN8200_SNIC_ACCEPT_SOCKET_FAIL,
	SN8200_SNIC_PARTIAL_CLOSE_FAIL,
	SN8200_SNIC_CONNECTION_PARTIALLY_CLOSED = 0x0A,
	SN8200_SNIC_CONNECTION_CLOSED,
	SN8200_SNIC_CLOSE_SOCKET_FAIL,
	SN8200_SNIC_PACKET_TOO_LARGE,
	SN8200_SNIC_SEND_FAIL,
	SN8200_SNIC_CONNECT_TO_SERVER_FAIL,
	SN8200_SNIC_NOT_ENOUGH_MEMORY = 0x10,
	SN8200_SNIC_TIMEOUT,
	SN8200_SNIC_CONNECTION_UP,
	SN8200_SNIC_GETSOCKOPT_FAIL,
	SN8200_SNIC_SETSOCKOPT_FAIL,
	SN8200_SNIC_INVALID_ARGUMENT,
	SN8200_SNIC_SEND_ARP_FAIL,
	SN8200_SNIC_INVALID_SOCKET,
	SN8200_SNIC_CONNECT_TO_SERVER_PENDING,
	SN8200_SNIC_SOCKET_NOT_BOUND,
	SN8200_SNIC_SOCKET_NOT_CONNECTED,
	SN8200_SNIC_NO_NETWORK = 0x20,
	SN8200_SNIC_INIT_NOT_DONE,
	SN8200_SNIC_NET_IF_FAIL,
	SN8200_SNIC_NET_IF_NOT_UP,
	SN8200_SNIC_DHCP_START_FAIL,
} SN8200_SNIC_RSP;

typedef enum {
	SN8200_WIFI_ON_REQ = 0,
	SN8200_WIFI_OFF_REQ,
	SN8200_WIFI_JOIN_REQ,
	SN8200_WIFI_DISCONNECT_REQ,
	SN8200_WIFI_GET_STATUS_REQ,
	SN8200_WIFI_SCAN_REQ,
	SN8200_WIFI_GET_STA_RSSI_REQ,
	SN8200_WIFI_AP_CTRL_REQ,

	SN8200_WIFI_NETWORK_STATUS_IND = 0x10,
	SN8200_WIFI_SCAN_RESULT_IND,
	SN8200_WIFI_RSSI_IND,

	SN8200_WIFI_ON_RSP = 0x80,
	SN8200_WIFI_OFF_RSP,
	SN8200_WIFI_JOIN_RSP,
	SN8200_WIFI_DISCONNECT_RSP,
	SN8200_WIFI_GET_STATUS_RSP,
	SN8200_WIFI_SCAN_RSP,
	SN8200_WIFI_GET_STA_RSSI_RSP,
	SN8200_WIFI_AP_CTRL_RSP,
} SN8200_WIFI;

typedef enum {
	SN8200_WIFI_MODE_WIFI_OFF,
	SN8200_WIFI_MODE_NO_NETWORK,
	SN8200_WIFI_MODE_STA_JOINED,
	SN8200_WIFI_MODE_AP_STARTED,
	SN8200_WIFI_MODE_SNIC_INIT_NOT_DONE,
	SN8200_WIFI_MODE_SNIC_INIT_DONE,
	/* Non-mode special values */
	SN8200_WIFI_MODE_LIST_END,
	SN8200_WIFI_MODE_ANY,
} SN8200_WIFI_MODE;


typedef struct {
	uint8_t ack;
	uint16_t l0;
	uint16_t l1;
	uint16_t length;
	uint8_t command;
	uint8_t* payload;
	uint8_t checksum;
} sn8200_frame;

volatile uint8_t rx_buffer[SN8200_DMA_RX_BUFFER];
uint8_t tx_buffer[SN8200_DMA_TX_BUFFER];
volatile uint8_t empty_buffer[SN8200_DMA_TX_BUFFER];
volatile uint16_t tx_buffered_amount;

// State stuff
volatile uint8_t receive_active;
volatile uint8_t write_waiting;
volatile uint8_t transfer_active;
volatile uint8_t active_buffer;

sn8200_frame pending_frame;
volatile uint16_t pending_read_index;

void SN8200_Init(void);
void SN8200_SendPacket(uint8_t ack_required, uint8_t command, uint8_t* payload, uint16_t length);
void SN8200_SendRaw(uint8_t value);
void SN8200_SendBuffer(uint8_t* buf, uint32_t len);
void SN8200_StartTransfer(void);
void SN8200_StopTransfer(void);
void SN8200_RecvPacket(uint8_t command, uint8_t* payload, uint16_t length);
