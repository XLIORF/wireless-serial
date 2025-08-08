#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "esp_crc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "portmacro.h"
#include "projdefs.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "rom/ets_sys.h"
#include "rom/crc.h"
#include "wlcon.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_timer.h"

#define CON_TYPE_RST 0x01
#define CON_TYPE_ACK 0x02
#define CON_TYPE_EST 0x03

static const char *TAG = "Serial_ESPNow";

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t target_mac[ESP_NOW_ETH_ALEN] = {0xff};

// 是否作为主机
static bool is_master = false;
// 无线通信状态
wireless_status_t status = WIRELESS_STATUS_BROADCAST;
// ESP-NOW 回调事件队列
static xQueueHandle espnow_cb_queue = NULL;
// 无线数据接收队列
static xQueueHandle wlcon_recv_queue = NULL;
static xQueueHandle wlcon_send_queue = NULL;
// 主机决定码
uint8_t master_ruling_code = 0;
// 心跳定时器
static esp_timer_handle_t heartbeat_timer = NULL;
// 心跳时间间隔
static uint32_t heartbeat_time = 0;
// 心跳发送间隔，防止心跳包发送过快
static uint32_t heartbeat_interval = 0;
// 任务优先级
static int wlcon_manager_priority = CONFIG_WLCON_MANAGER_PRORITY;
// 任务句柄
static TaskHandle_t wlcon_manager_handle = NULL;

// 静态分配数据包，避免重复的IO操作
static wireless_packet_t *broadcast_packet = NULL,
                         *connect_rst_packet = NULL,
                         *connect_ack_packet = NULL,
                         *connect_establish_packet = NULL,
                         *heartbeat_packet = NULL,
                         *heartbeat_ack_packet = NULL,
                         *data_ack_packet = NULL;

static size_t bp_len = sizeof(wireless_packet_t) + 1,
              crp_len = sizeof(wireless_packet_t) + 2,
              cap_len = sizeof(wireless_packet_t) + 2,
              cep_len = sizeof(wireless_packet_t) + 2,
              hp_len = sizeof(wireless_packet_t),
              hap_len = sizeof(wireless_packet_t),
              dap_len = sizeof(wireless_packet_t);

#define free_p(p)        \
    if (p != NULL)       \
    {                    \
        free((void *)p); \
        p = NULL;        \
    }

void destroy_packet()
{
    free_p(broadcast_packet);
    free_p(connect_rst_packet);
    free_p(connect_ack_packet);
    free_p(connect_establish_packet);
    free_p(heartbeat_packet);
    free_p(heartbeat_ack_packet);
    free_p(data_ack_packet);
}

bool wlcon_create_packet()
{
    // 广播包
    broadcast_packet = malloc(bp_len);
    connect_rst_packet = malloc(crp_len);
    connect_ack_packet = malloc(cap_len);
    connect_establish_packet = malloc(cep_len);
    heartbeat_packet = malloc(hp_len);
    heartbeat_ack_packet = malloc(hap_len);
    data_ack_packet = malloc(dap_len);
    if (broadcast_packet == NULL || connect_rst_packet == NULL || connect_ack_packet == NULL || connect_establish_packet == NULL || heartbeat_packet == NULL || heartbeat_ack_packet == NULL || data_ack_packet == NULL)
    {
        ESP_LOGE(TAG, "Malloc packet fail");
        destroy_packet();
        return false;
    }
    broadcast_packet->type = WIRELESS_PACKET_TYPE_BROADCAST;
    broadcast_packet->length = 1;
    broadcast_packet->version = WIRELESS_PACKET_VERSION;
    broadcast_packet->crc = 0;
    broadcast_packet->payload[0] = master_ruling_code;
    broadcast_packet->crc = crc16_le(UINT16_MAX, (uint8_t const *)broadcast_packet, sizeof(wireless_packet_t) + 1);
    // 连接包
    connect_rst_packet->type = WIRELESS_PACKET_TYPE_CONNECT;
    connect_rst_packet->length = 2;
    connect_rst_packet->version = WIRELESS_PACKET_VERSION;
    connect_rst_packet->crc = 0;
    connect_rst_packet->payload[0] = 1;
    connect_rst_packet->payload[1] = 0;
    connect_rst_packet->crc = 0;

    connect_ack_packet->type = WIRELESS_PACKET_TYPE_CONNECT;
    connect_ack_packet->length = 2;
    connect_ack_packet->version = WIRELESS_PACKET_VERSION;
    connect_ack_packet->crc = 0;
    connect_ack_packet->payload[0] = 2;
    connect_ack_packet->payload[1] = 0;
    connect_ack_packet->crc = 0;

    connect_establish_packet->type = WIRELESS_PACKET_TYPE_CONNECT;
    connect_establish_packet->length = 2;
    connect_establish_packet->version = WIRELESS_PACKET_VERSION;
    connect_establish_packet->crc = 0;
    connect_establish_packet->payload[0] = 3;
    connect_establish_packet->payload[1] = 0;
    connect_establish_packet->crc = 0;

    heartbeat_packet->type = WIRELESS_PACKET_TYPE_DATA;
    heartbeat_packet->length = 0;
    heartbeat_packet->version = WIRELESS_PACKET_VERSION;
    heartbeat_packet->crc = 0;
    heartbeat_packet->crc = crc16_le(UINT16_MAX, (uint8_t const *)heartbeat_packet, hp_len);

    heartbeat_ack_packet->type = WIRELESS_PACKET_TYPE_DATA_ACK;
    heartbeat_ack_packet->length = 0;
    heartbeat_ack_packet->version = WIRELESS_PACKET_VERSION;
    heartbeat_ack_packet->crc = 0;
    heartbeat_ack_packet->crc = crc16_le(UINT16_MAX, (uint8_t const *)heartbeat_ack_packet, hap_len);

    data_ack_packet->type = WIRELESS_PACKET_TYPE_DATA_ACK;
    data_ack_packet->length = 0;
    data_ack_packet->version = WIRELESS_PACKET_VERSION;
    data_ack_packet->crc = 0;
    data_ack_packet->crc = crc16_le(UINT16_MAX, (uint8_t const *)data_ack_packet, dap_len);
    return true;
}

/**
 * @brief ESP-NOW发送回调函数
 *
 * 当ESP-NOW数据发送完成时，系统会调用此回调函数来通知发送结果。
 * 该函数会将发送结果封装成事件并发送到espnow_cb_queue队列中。
 *
 * @param mac_addr 接收设备的MAC地址指针
 * @param status 发送状态，表示数据发送成功或失败
 */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    // 检查MAC地址参数有效性
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    // 检查队列句柄有效性
    if (espnow_cb_queue == NULL)
    {
        ESP_LOGE(TAG, "Send cb error: espnow_cb_queue is NULL");
        return;
    }
    // 封装发送回调事件信息
    evt.id = ESPNOW_SEND_CB;
    // memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    // 将事件发送到队列中
    if (xQueueSend(espnow_cb_queue, &evt, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Func[espnow_send_cb] Send queue fail");
    }
}

/**
 * @brief ESP-NOW数据接收回调函数
 *
 * 当ESP-NOW接收到数据时，该回调函数会被调用。函数将接收的数据封装成事件结构体，
 * 并发送到ESP-NOW事件队列中供其他任务处理。
 *
 * @param mac_addr 源设备的MAC地址指针
 * @param data 接收到的数据指针
 * @param len 接收数据的长度
 *
 * @return 无返回值
 */
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    /* 参数合法性检查 */
    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }
    if (espnow_cb_queue == NULL)
    {
        ESP_LOGE(TAG, "Send cb error: espnow_cb_queue is NULL");
        return;
    }
    /* 为接收数据分配内存空间 */
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    evt.id = ESPNOW_RECV_CB;
    recv_cb->len = len;
    memcpy(recv_cb->data, data, len);
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    /* 将接收事件发送到队列中 */
    if (xQueueSend(espnow_cb_queue, &evt, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Func[espnow_recv_cb] Send queue fail");
    }
    // free(recv_cb->data);
}

// 封装数据包发送函数
static inline bool send_broadcast_packet()
{
    if (esp_now_send(broadcast_mac, (uint8_t *)broadcast_packet, bp_len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send broadcast packet fail");
        return false;
    }

    return true;
}

static inline bool send_connect_packet(int type, uint8_t connect_code)
{
    wireless_packet_t *s_packet = NULL;
    size_t sp_len = 0;
    if (type == CON_TYPE_RST)
    {
        s_packet = connect_rst_packet;
        sp_len = crp_len;
    }
    else if (type == CON_TYPE_ACK)
    {
        s_packet = connect_ack_packet;
        sp_len = cap_len;
    }
    else
    {
        s_packet = connect_establish_packet;
        sp_len = cep_len;
    }
    s_packet->payload[1] = connect_code;
    s_packet->crc = 0;
    s_packet->crc = crc16_le(UINT16_MAX, (uint8_t *)s_packet, sp_len);
    if (esp_now_send(target_mac, (uint8_t *)s_packet, sp_len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send connecting packet fail");
        return false;
    }
    return true;
}

static inline bool send_heartbeat_packet(int type)
{
    if (esp_now_send(target_mac, (uint8_t *)(type == 1 ? heartbeat_packet : heartbeat_ack_packet), hp_len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send heartbeat packet fail");
        return false;
    }
    return true;
}

static inline bool send_ack_packet()
{
    if (esp_now_send(target_mac, (uint8_t *)data_ack_packet, dap_len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send ack packet fail");
        return false;
    }
    return true;
}

// 优化后的CRC校验宏
#define CRC_CHECK(packet) ({                                                                                 \
    uint16_t crc_recv = packet->crc;                                                                         \
    packet->crc = 0;                                                                                         \
    uint16_t crc_calc = crc16_le(UINT16_MAX, (uint8_t *)packet, sizeof(wireless_packet_t) + packet->length); \
    packet->crc = crc_recv;                                                                                  \
    crc_calc == crc_recv;                                                                                    \
})

void wireless_add_peer(const uint8_t *mac_addr, bool encrypt)
{

    if (esp_now_is_peer_exist(mac_addr) == false)
    {
        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
        if (peer == NULL)
        {
            ESP_LOGE(TAG, "Malloc peer information fail");
            vTaskDelete(NULL);
        }
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = CONFIG_ESPNOW_CHANNEL;
        peer->ifidx = ESPNOW_WIFI_IF;
        peer->encrypt = encrypt;
        if (encrypt)
            memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
        memcpy(peer->peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
        free(peer);
        // 记录对方MAC地址
        memcpy(target_mac, mac_addr, ESP_NOW_ETH_ALEN);
    }
}

static inline bool wireless_packet_check(wireless_packet_t *packet)
{
    // 检查数据包长度
    if (packet == NULL) // ||packet->length > WIRELESS_PACKET_MAX_PAYLOAD_SIZE)
    {
        ESP_LOGE(TAG, "Invalid packet length: %d", packet->length);
        return false;
    }
    // 检查版本号
    if (packet->version != WIRELESS_PACKET_VERSION)
    {
        ESP_LOGE(TAG, "Unsupported packet version: %d", packet->version);
        return false;
    }
    // CRC校验
    if (!CRC_CHECK(packet))
    {
        ESP_LOGE(TAG, "CRC check failed");
        return false;
    }
    return true;
}

void wlcon_con_manager(void *pvParameters)
{
    // 连接重试次数
    int retry_count = 0;
    uint32_t last_broadcast_time = 0;
    uint32_t last_connect_rst_time = 0;
    // 每一次发起连接的代码，标记不同的连接包
    uint8_t connect_code = 0;
    // 接受回调事件
    espnow_event_t evt = {0};
    // 接受外部输入数据
    buf_len_t buflen = {0};
    // 等待上一个数据包的应答包
    bool wait_ack = false;
    esp_timer_start_periodic(heartbeat_timer, 1000 * 10); // 设置心跳间隔为配置的心跳间隔
    while (1)
    {
        if (status == WIRELESS_STATUS_CONNECTED)
        {
            if (heartbeat_time > (CONFIG_HEARTBEAT_INTERVAL + 1000))
            {
                // 心跳超时, 连接断开
                printf("心跳超时, 连接断开\n");
                printf("heartbeat_time: %d ms, %d ms\n", heartbeat_time, (CONFIG_HEARTBEAT_INTERVAL + 1500));
                status = WIRELESS_STATUS_DISCONNECTED;
            }

            // 处理数据发送
            if (!wait_ack && wlcon_send_queue != NULL && xQueueReceive(wlcon_send_queue, &buflen, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                wait_ack = true;
                size_t plen = sizeof(wireless_packet_t) + buflen.len;
                wireless_packet_t *wp = malloc(plen);
                if (wp == NULL)
                {
                    ESP_LOGE(TAG, "内存分配失败!");
                    continue;
                }
                wp->version = WIRELESS_PACKET_VERSION;
                wp->type = WIRELESS_PACKET_TYPE_DATA;
                wp->length = buflen.len;
                wp->crc = 0;
                memcpy(wp->payload, buflen.buf, buflen.len);
                wp->crc = crc16_le(UINT16_MAX, (uint8_t const *)wp, plen);
                if (esp_now_send(target_mac, (uint8_t *)wp, plen) != ESP_OK)
                {
                    ESP_LOGE(__FUNCTION__, "Send data packet fail");
                    wait_ack = false;
                }
                free(wp);
                if ((buflen.flag & 0x01) == 0x01)
                {
                    free(buflen.buf);
                }
            }
        }
        else if (status == WIRELESS_STATUS_DISCONNECTED)
        {
            // 清理数据，再次广播
            printf("Disconnected.\n");
            // 删除对端设备
            if (!IS_BROADCAST_ADDR(target_mac))
            {
                esp_now_del_peer(target_mac);
                memcpy(target_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
            }
            retry_count = 0;
            is_master = false;
            status = WIRELESS_STATUS_BROADCAST;
        }
        else if (status == WIRELESS_STATUS_BROADCAST)
        {
            // 继续广播
            if (xTaskGetTickCount() - last_broadcast_time > pdMS_TO_TICKS(CONFIG_BROADCAST_INTERVAL))
            {
                send_broadcast_packet();
                last_broadcast_time = xTaskGetTickCount();
            }
        }
        else if (status == WIRELESS_STATUS_CONNECT_RST)
        {
            if (xTaskGetTickCount() - last_connect_rst_time >= pdMS_TO_TICKS(CONFIG_CONNECT_INTERVAL))
            {
                if (retry_count >= CONFIG_CONNECT_RETRY)
                {
                    // 连接失败, 继续广播
                    status = WIRELESS_STATUS_BROADCAST;
                    retry_count = 0;
                    is_master = false;
                }
                connect_code = esp_random() & 0xff;
                send_connect_packet(1, connect_code);
                retry_count++;
                last_connect_rst_time = xTaskGetTickCount();
            }
        }
        // 回调处理
        if (xQueueReceive(espnow_cb_queue, &evt, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            // 收到数据包
            if (evt.id == ESPNOW_RECV_CB)
            {
                wireless_packet_t *packet = (wireless_packet_t *)evt.info.recv_cb.data;
                // 有效检测
                if (!wireless_packet_check(packet))
                {
                    ESP_LOGE(TAG, "Invalid packet received");
                    free_p(packet);
                    continue; // 多个不相关动作在同一循环中时，慎用continue和break
                }
                // 数据包分类处理
                switch (packet->type)
                {
                    // 广播包, 用于设备发现，只在广播状态下处理
                case WIRELESS_PACKET_TYPE_BROADCAST:
                    if (status != WIRELESS_STATUS_BROADCAST)
                    { // 收到广播包是已连接地址发出的，则判定为连接断开
                        if (memcmp(evt.info.recv_cb.mac_addr, target_mac, ESP_NOW_ETH_ALEN) == 0)
                        {
                            // 对端进入广播状态，判定对方掉线重新连接
                            status = WIRELESS_STATUS_DISCONNECTED;
                        }
                        break;
                    }
                    if (master_ruling_code > packet->payload[0])
                    {
                        is_master = true;
                        wireless_add_peer(evt.info.recv_cb.mac_addr, false);
                        // 停止广播
                        status = WIRELESS_STATUS_CONNECT_RST;
                    }
                    break;
                // 连接包, 用于连接建立，只在广播状态下处理
                case WIRELESS_PACKET_TYPE_CONNECT:
                    if (status != WIRELESS_STATUS_BROADCAST && status != WIRELESS_STATUS_CONNECT_RST)
                    {
                        break;
                    }
                    // 判断是请求包还是应答包
                    if (packet->payload[0] == CON_TYPE_RST) // 请求包
                    {
                        wireless_add_peer(evt.info.recv_cb.mac_addr, false);
                        connect_code = packet->payload[1];
                        send_connect_packet(2, connect_code + 1);
                    }
                    else if (packet->payload[0] == CON_TYPE_ACK) // 应答包
                    {
                        // 验证连接校验码
                        if (packet->payload[1] == connect_code + 1)
                        {
                            send_connect_packet(3, packet->payload[1] + 1);
                            // 进入连接状态
                            is_master = true;
                            printf("Connected.\n");
                            // 重新建立加密连接
                            esp_now_del_peer(target_mac);
                            wireless_add_peer(evt.info.recv_cb.mac_addr, true);
                            portENTER_CRITICAL();
                            heartbeat_time = 0;
                            portEXIT_CRITICAL();
                            status = WIRELESS_STATUS_CONNECTED;
                        }
                        else
                        {
                            status = WIRELESS_STATUS_BROADCAST;
                        }
                    }
                    else if (packet->payload[0] == CON_TYPE_EST) // 连接建立包
                    {
                        if (packet->payload[1] == connect_code + 2)
                        {
                            // 进入连接状态
                            is_master = false;
                            status = WIRELESS_STATUS_CONNECTED;
                            portENTER_CRITICAL();
                            heartbeat_time = 0;
                            portEXIT_CRITICAL();
                            printf("Connected.\n");
                            esp_now_del_peer(target_mac);
                            wireless_add_peer(evt.info.recv_cb.mac_addr, true);
                        }
                        else
                        {
                            status = WIRELESS_STATUS_BROADCAST;
                        }
                    }
                    break;
                    // 数据包，用于数据传输，只在连接状态下处理
                case WIRELESS_PACKET_TYPE_DATA:
                    if (status != WIRELESS_STATUS_CONNECTED)
                    {
                        ESP_LOGD(TAG, "未连接状态下收到数据包，丢弃数据包");
                        break;
                    }
                    portENTER_CRITICAL();
                    heartbeat_time = 0;
                    portEXIT_CRITICAL();
                    if (packet->length == 0)
                    {
                        // 如果数据包长度为0，是心跳包，回复应答
                        send_heartbeat_packet(2);
                        break;
                    }
                    buf_len_t espnow_serial = {
                        .len = packet->length,
                        .buf = malloc(packet->length),
                    };
                    memcpy(espnow_serial.buf, packet->payload, packet->length);
                    // 发送应答包
                    send_ack_packet();
                    if (wlcon_recv_queue == NULL || xQueueSend(wlcon_recv_queue, &espnow_serial, pdMS_TO_TICKS(10)) != pdTRUE)
                    {
                        ESP_LOGE(TAG, "输出数据到串口队列失败");
                        break;
                    }
                    break;
                    // 数据应答包，用于数据发送成功的确认，只在连接状态下处理
                case WIRELESS_PACKET_TYPE_DATA_ACK:
                    if (status != WIRELESS_STATUS_CONNECTED)
                    {
                        ESP_LOGD(TAG, "未连接状态下收到数据应答包，丢弃应答包");
                        break;
                    }
                    wait_ack = false;
                    portENTER_CRITICAL();
                    heartbeat_time = 0;
                    portEXIT_CRITICAL();
                    break;
                }
                // 释放数据包内存，此内存在espnow_recv_cb中分配
                free_p(packet);
            }
        }
    }
}

void wlcon_heartbeat_handler(void *pvParameters)
{
    heartbeat_time += 10;
    heartbeat_interval += 10;
    if (is_master && status == WIRELESS_STATUS_CONNECTED && heartbeat_time > (CONFIG_HEARTBEAT_INTERVAL))
    {
        // 发送心跳包
        if (heartbeat_interval > (CONFIG_HEARTBEAT_INTERVAL))
        {
            heartbeat_interval = 0; // 重置心跳发送间隔
            send_heartbeat_packet(1);
        }
    }
}

void wifi_init(void)
{
    static bool initialized = false;
    if (initialized)
    {
        return;
    }
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0));

    initialized = true;
}

void wlcon_io_register(xQueueHandle send, xQueueHandle recv)
{
    wlcon_send_queue = send;
    wlcon_recv_queue = recv;
    if (wlcon_send_queue == NULL || wlcon_recv_queue == NULL)
    {
        ESP_LOGW(TAG, "There are unconfigured input and output queues.");
    }
}

bool wlcon_is_connected()
{
    if (status != WIRELESS_STATUS_CONNECTED)
        return false;
    return true;
}

esp_err_t wlcon_init(void)
{
    // 创建队列
    espnow_cb_queue = xQueueCreate(ESPNOW_CB_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_cb_queue == NULL)
    {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }
    wlcon_create_packet();
    // 初始化ESP_NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    // 添加广播地址为peer
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(espnow_cb_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    // 创建心跳定时器
    esp_timer_init();
    esp_timer_create_args_t timer_args = {
        .callback = &wlcon_heartbeat_handler, // 心跳定时器回调函数
        .name = "heartbeat_timer",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
    };
    esp_err_t ret = esp_timer_create(&timer_args, &heartbeat_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Create heartbeat timer fail: %s", esp_err_to_name(ret));
        return ret;
    }
    // 获取ESP_NOW版本
    uint32_t esp_now_version;
    esp_now_get_version(&esp_now_version);
    ESP_LOGI(TAG, "ESP-NOW Version: %d", esp_now_version);

    master_ruling_code = esp_random() & 0xff;

    xTaskCreate(wlcon_con_manager, "wlcon_con_manager", 2048, NULL, wlcon_manager_priority, &wlcon_manager_handle);
    // 开始广播
    printf("Broadcast...\n");
    return ESP_OK;
}
