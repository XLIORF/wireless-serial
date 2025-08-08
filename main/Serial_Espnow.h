#ifndef __ESPNOW_H__
#define __ESPNOW_H__
#if 1 // CONFIG_STATION_MODE
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF ESP_IF_WIFI_AP
#endif

#define ESPNOW_CB_QUEUE_SIZE 8
#define WIRELESS_RECV_QUEUE_SIZE 12
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

#define BUF_SIZE (1024)
#define RX_BUF_SIZE (32)
#define EX_UART_NUM UART_NUM_0
#include "esp_system.h"
// 无线通信状态
typedef enum
{
    WIRELESS_STATUS_NONE = 0,      // 无状态
    WIRELESS_STATUS_BROADCAST = 1, // 广播状态
    WIRELESS_STATUS_CONNECT_RST = 2,
    WIRELESS_STATUS_CONNECTED = 3,
    WIRELESS_STATUS_DISCONNECTED = 4,
    WIRELESS_STATUS_MAX_INDEX = 5,
} wireless_status_t;

// 无线数据包类型
typedef enum
{
    WIRELESS_PACKET_TYPE_BROADCAST = 0, // 广播包
    WIRELESS_PACKET_TYPE_CONNECT,       // 连接包
    WIRELESS_PACKET_TYPE_DATA,          // 数据包
    WIRELESS_PACKET_TYPE_DATA_ACK,      // 数据应答包
    // WIRELESS_PACKET_TYPE_MAX_INDEX,
} wireless_packet_type_t;

// 无线数据包格式
typedef struct
{
    uint32_t version; // 版本
    wireless_packet_type_t type;
    uint32_t length;    // 数据长度
    uint16_t crc;       // 校验和
    uint8_t payload[0]; // 数据
} __attribute__((packed)) wireless_packet_t;

typedef struct
{
    uint16_t len;
    uint8_t *data; // 必须可以被free正确释放
    uint8_t *mac;
} __attribute__((packed)) espnow_packet_t;

typedef struct
{
    uint16_t len;
    uint8_t *data; // 必须可以被free正确释放
} __attribute__((packed)) buf_len_t;

typedef enum
{
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB
} espnow_event_id_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int len;
} espnow_event_recv_cb_t;

typedef union
{
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

typedef struct
{
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

void wifi_init(void);
esp_err_t Serial_Espnow_init(void);
void uart_rx_task(void *);
#endif