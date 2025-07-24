#ifndef __ESPNOW_H__
#define __ESPNOW_H__
#if 1 //CONFIG_STATION_MODE
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           12

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

#define BUF_SIZE (1024)
#define RX_BUF_SIZE (32)
#define EX_UART_NUM UART_NUM_0

typedef enum {
    ESPNOW_DATA_BROADCAST = 0,
    ESPNOW_DATA_UNICAST = 1,
    ESPNOW_DATA_ACK = 2,
    ESPNOW_DATA_MAX = 3,
} espnow_type_t;

typedef struct {
    espnow_type_t type;
    uint16_t crc;
    // uint8_t s_seq;
    uint8_t payload[0];
} __attribute__((packed)) espnow_package_t;

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB
}espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

esp_err_t espnow_send_package(void *, uint8_t , bool );
void wifi_init(void);
esp_err_t Serial_Espnow_init(void);
void espnow_task(void *);
void uart_rx_task(void *);
#endif