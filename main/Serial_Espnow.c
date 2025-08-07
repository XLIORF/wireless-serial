#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
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
#include "Serial_Espnow.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_timer.h"

#define WIRELESS_PACKET_VERSION 1U
static const char *TAG = "Serial_ESPNow";

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t target_mac[ESP_NOW_ETH_ALEN] = {0xff};

// 等待上一个数据包的应答包
static bool wait_ack = false;
// 是否作为主机
static bool is_master = false;
// 无线通信状态
wireless_status_t status = WIRELESS_STATUS_BROADCAST;
// ESP-NOW 回调事件队列
static xQueueHandle espnow_cb_queue;
// 无线数据接收队列
static xQueueHandle wireless_recv_queue;
// 主机决定码
uint8_t master_ruling_code;
// 心跳定时器
static esp_timer_handle_t heartbeat_timer = NULL;
// 心跳时间间隔
static uint32_t heartbeat_time = 0;
// 心跳发送间隔，防止心跳包发送过快
static uint32_t heartbeat_interval = 0;
// 任务优先级
static int wireless_manager_task_proiority = CONFIG_WLCON_MANAGER_PRORITY;
// 任务句柄
static TaskHandle_t wireless_manager_task_handle = NULL;

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
    ESP_LOGI(TAG, "队列剩余空间: %d", uxQueueSpacesAvailable(espnow_cb_queue));
    /* 将接收事件发送到队列中 */
    if (xQueueSend(espnow_cb_queue, &evt, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Func[espnow_recv_cb] Send queue fail");
    }
    // free(recv_cb->data);
}

// 封装数据包发送函数
static bool send_broadcast_packet()
{
    size_t p_len = sizeof(wireless_packet_t) + 1;
    wireless_packet_t *packet = malloc(p_len);
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "Malloc packet fail");
        return false;
    }
    packet->type = WIRELESS_PACKET_TYPE_BROADCAST;
    packet->length = 1;
    packet->version = WIRELESS_PACKET_VERSION;
    packet->crc = 0;
    packet->payload[0] = master_ruling_code;
    packet->crc = crc16_le(UINT16_MAX, (uint8_t const *)packet, sizeof(wireless_packet_t) + 1);

    if (esp_now_send(broadcast_mac, (uint8_t *)packet, p_len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send broadcast packet fail");
        free(packet);
        return false;
    }

    return true;
}

static bool send_connect_packet(int type, uint8_t connect_code)
{
    size_t p_len = sizeof(wireless_packet_t) + 2;
    wireless_packet_t *packet = malloc(p_len);
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "Malloc packet fail");
        return false;
    }
    packet->type = WIRELESS_PACKET_TYPE_CONNECT;
    packet->length = 2;
    packet->version = WIRELESS_PACKET_VERSION;
    packet->crc = 0;
    packet->payload[0] = type;
    packet->payload[1] = connect_code;
    packet->crc = crc16_le(UINT16_MAX, (uint8_t const *)packet, p_len);
    if (esp_now_send(target_mac, (uint8_t *)packet, p_len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send connecting packet fail");
        free(packet);
        return false;
    }
    return true;
}

static bool send_heartbeat_packet(int type)
{
    size_t p_len = sizeof(wireless_packet_t);
    wireless_packet_t *packet = malloc(p_len);
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "Malloc packet fail");
        return false;
    }
    packet->type = type == 1 ? WIRELESS_PACKET_TYPE_DATA : WIRELESS_PACKET_TYPE_DATA_ACK;
    packet->length = 0;
    packet->version = WIRELESS_PACKET_VERSION;
    packet->crc = 0;
    packet->crc = crc16_le(UINT16_MAX, (uint8_t const *)packet, p_len);
    if (esp_now_send(target_mac, (uint8_t *)packet, p_len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send heartbeat packet fail");
        free(packet);
        return false;
    }
    return true;
}

static bool send_ack_packet()
{
    size_t p_len = sizeof(wireless_packet_t);
    wireless_packet_t *packet = malloc(p_len);
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "Malloc packet fail");
        return false;
    }
    packet->type = WIRELESS_PACKET_TYPE_DATA_ACK;
    packet->length = 0;
    packet->version = WIRELESS_PACKET_VERSION;
    packet->crc = 0;
    packet->crc = crc16_le(UINT16_MAX, (uint8_t const *)packet, p_len);
    if (esp_now_send(target_mac, (uint8_t *)packet, p_len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send ack packet fail");
        free(packet);
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

inline bool wireless_packet_check(wireless_packet_t *packet)
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

void wireless_con_manager_task(void *pvParameters)
{
    int retry_count = 0;
    uint32_t last_broadcast_time = 0;
    uint32_t last_connect_rst_time = 0;
    uint8_t connect_code = 0;
    espnow_event_t evt;
    esp_timer_start_periodic(heartbeat_timer, 1000 * 10); // 设置心跳间隔为配置的心跳间隔
    while (1)
    {
        if (status == WIRELESS_STATUS_CONNECTED)
        {
            if (heartbeat_time > (CONFIG_HEARTBEAT_INTERVAL + 3000))
            {
                // 心跳超时, 连接断开
                printf("心跳超时, 连接断开\n");
                printf("heartbeat_time: %d ms, %d ms\n", heartbeat_time, (CONFIG_HEARTBEAT_INTERVAL + 1500));
                status = WIRELESS_STATUS_DISCONNECTED;
            }
            // else
        }
        else if (status == WIRELESS_STATUS_DISCONNECTED)
        {
            // TODO 清理数据，再次广播
            printf("设备掉线\n");
            retry_count = 0;
            is_master = false;
            status = WIRELESS_STATUS_BROADCAST;
        }
        else if (status == WIRELESS_STATUS_BROADCAST)
        {
            // 继续广播
            if (xTaskGetTickCount() - last_broadcast_time > pdMS_TO_TICKS(CONFIG_BROADCAST_INTERVAL))
            {
                printf("Broadcast...\n");
                send_broadcast_packet();
                last_broadcast_time = xTaskGetTickCount();
            }
        }
        else if (status == WIRELESS_STATUS_CONNECT_RST)
        {
            if (xTaskGetTickCount() - last_connect_rst_time < pdMS_TO_TICKS(CONFIG_CONNECT_INTERVAL))
            {
                continue;
            }
            if (retry_count >= 3)
            {
                // 连接失败, 继续广播
                status = WIRELESS_STATUS_BROADCAST;
                retry_count = 0;
                is_master = false;
            }
            connect_code = esp_random() & 0xff;
            send_connect_packet(1, connect_code);
            last_connect_rst_time = xTaskGetTickCount();
            retry_count++;
        }

        // 回调处理
        if (xQueueReceive(espnow_cb_queue, &evt, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            // 收到数据包
            if (evt.id == ESPNOW_RECV_CB)
            {
                // 有效检测
                wireless_packet_t *packet = (wireless_packet_t *)evt.info.recv_cb.data;
                if (!wireless_packet_check(packet))
                {
                    ESP_LOGE(TAG, "Invalid packet received");
                    free(packet);
                    continue;
                }
                if (status != WIRELESS_STATUS_CONNECTED && !(packet->type == WIRELESS_PACKET_TYPE_BROADCAST || packet->type == WIRELESS_PACKET_TYPE_CONNECT))
                {
                    ESP_LOGD(TAG, "未连接状态下收到非广播包，丢弃数据包");
                    continue;
                }
                // 数据包分类处理
                switch (packet->type)
                {
                    // 广播包, 用于设备发现，只在广播状态下处理
                case WIRELESS_PACKET_TYPE_BROADCAST:
                    if (status != WIRELESS_STATUS_BROADCAST)
                    {
                        break;
                    }
                    if (master_ruling_code > packet->payload[0])
                    {
                        printf("作为主机端\n");
                        is_master = true;
                        wireless_add_peer(evt.info.recv_cb.mac_addr, false);
                        // 停止广播
                        status = WIRELESS_STATUS_CONNECT_RST;
                    }
                    break;
                // 连接包, 用于连接建立，只在广播状态下处理
                case WIRELESS_PACKET_TYPE_CONNECT:
                    if (status != WIRELESS_STATUS_BROADCAST)
                    {
                        break;
                    }
                    wireless_add_peer(evt.info.recv_cb.mac_addr, false);
                    // 判断是请求包还是应答包
                    if (packet->payload[0] == 0x01) // 请求包
                    {
                        connect_code = packet->payload[1];
                        send_connect_packet(2, connect_code + 1);
                        ESP_LOGD(TAG, "收到连接请求包，返回连接码: %d", connect_code);
                    }
                    else if (packet->payload[0] == 0x02) // 应答包
                    {
                        // 验证连接校验码
                        ESP_LOGD(TAG, "收到连接请求应答包，连接码: %d", packet->payload[1]);
                        if (packet->payload[1] == connect_code + 1)
                        {
                            send_connect_packet(3, packet->payload[1] + 1);
                            // 进入连接状态
                            is_master = true;
                            printf("连接建立成功\n");
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
                            printf("应答包校验码错误\n");
                            status = WIRELESS_STATUS_BROADCAST;
                        }
                    }
                    else if (packet->payload[0] == 0x03) // 连接建立包
                    {
                        ESP_LOGD(TAG, "收到连接建立包，连接码: %d", packet->payload[1]);
                        if (packet->payload[1] == connect_code + 2)
                        {
                            // 进入连接状态
                            is_master = false;
                            status = WIRELESS_STATUS_CONNECTED;
                            portENTER_CRITICAL();
                            heartbeat_time = 0;
                            portEXIT_CRITICAL();
                            printf("连接建立成功\n");
                            esp_now_del_peer(target_mac);
                            wireless_add_peer(evt.info.recv_cb.mac_addr, true);
                        }
                        else
                        {
                            printf("连接建立包校验码错误\n");
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
                        .data = malloc(packet->length),
                    };
                    memcpy(espnow_serial.data, packet->payload, packet->length);
                    // 发送应答包
                    send_ack_packet();
                    if (xQueueSend(wireless_recv_queue, &espnow_serial, pdMS_TO_TICKS(10)) != pdTRUE)
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
                    // TODO 数据发送成功，从队列中移除
                    wait_ack = false;
                    portENTER_CRITICAL();
                    heartbeat_time = 0;
                    portEXIT_CRITICAL();
                    break;
                }
                // 释放数据包内存，此内存在espnow_recv_cb中分配
                if (packet != NULL)
                {
                    free(packet);
                    packet = NULL;
                }
            }
        }
    }
}

void uart_rx_task(void *param)
{
    uint8_t data[RX_BUF_SIZE] = {0};
    size_t rx_len = 0;
    buf_len_t wireless_data;
    while (1)
    {
        vTaskDelay(5);
        if (xQueueReceive(wireless_recv_queue, &wireless_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            // 有数据接收
            if (wireless_data.len > 0 && wireless_data.data != NULL)
            {
                ESP_LOGD(TAG, "recv [esp_now->serial]:%s", (char *)wireless_data.data);
                uart_write_bytes(EX_UART_NUM, (const char *)wireless_data.data, wireless_data.len);
                free(wireless_data.data);
            }
        }
        // 判断用户是否输入
        esp_err_t err = uart_get_buffered_data_len(EX_UART_NUM, &rx_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error: %d", err);
            continue;
        }
        if (rx_len <= 0)
            continue;
        // 未连接但是有用户输入，则提示输入无效
        if (status != WIRELESS_STATUS_CONNECTED)
        {
            // 直接清除输入缓冲区
            uart_flush_input(EX_UART_NUM);
            uart_write_bytes(EX_UART_NUM, "未连接输入无效\r\n", strlen("未连接输入无效\r\n"));
            continue;
        }
        // 读取输入并转发
        int serial_rx_len = uart_read_bytes(EX_UART_NUM, data, RX_BUF_SIZE, 0);
        if (serial_rx_len > 0)
        {
            ESP_LOGD(TAG, "send [serial->esp_now]:%s", (char *)data);
            // 发送数据包
            size_t len = sizeof(wireless_packet_t) + serial_rx_len;
            wireless_packet_t *wp = malloc(len);
            if (wp == NULL)
            {
                ESP_LOGE(TAG, "内存分配失败!");
                continue;
            }
            wp->version = WIRELESS_PACKET_VERSION;
            wp->type = WIRELESS_PACKET_TYPE_DATA;
            wp->length = serial_rx_len;
            wp->crc = 0;
            memcpy(wp->payload, data, serial_rx_len);
            wp->crc = crc16_le(UINT16_MAX, (uint8_t const *)wp, len);
            if (esp_now_send(target_mac, (uint8_t *)wp, len) != ESP_OK)
            {
                ESP_LOGE(TAG, "Send data packet fail");
                free(wp);
            }
        }
    }
}

void wireless_heartbeat_handler(void *pvParameters)
{
    // TODO 加一个锁，看看是不是主机收到心跳应答包时恰好除法定时器中断导致心跳时间没有重置引起的连接断开
    heartbeat_time += 10;
    heartbeat_interval += 10;
    if (is_master && status == WIRELESS_STATUS_CONNECTED && heartbeat_time > (CONFIG_HEARTBEAT_INTERVAL / 2))
    {
        // 发送心跳包
        if (heartbeat_interval > (CONFIG_HEARTBEAT_INTERVAL / 2))
        {
            heartbeat_interval = 0; // 重置心跳发送间隔
            send_heartbeat_packet(1);
        }
    }
}

void wireless_heartbeat_task(void *pvParameters)
{
    while (1)
    {
        heartbeat_time += 10;
        heartbeat_interval += 10;
        if (is_master && status == WIRELESS_STATUS_CONNECTED && heartbeat_time > (CONFIG_HEARTBEAT_INTERVAL / 2))
        {
            // 发送心跳包
            if (heartbeat_interval > (CONFIG_HEARTBEAT_INTERVAL / 2))
            {
                heartbeat_interval = 0; // 重置心跳发送间隔
                send_heartbeat_packet(1);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
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

esp_err_t Serial_Espnow_init(void)
{
    // 创建队列
    espnow_cb_queue = xQueueCreate(ESPNOW_CB_QUEUE_SIZE, sizeof(espnow_event_t));
    wireless_recv_queue = xQueueCreate(WIRELESS_RECV_QUEUE_SIZE, sizeof(buf_len_t));
    if (espnow_cb_queue == NULL || wireless_recv_queue == NULL)
    {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

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
        .callback = &wireless_heartbeat_handler, // 心跳定时器回调函数
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

    xTaskCreate(wireless_con_manager_task, "wireless_con_manager_task", 2048, NULL, wireless_manager_task_proiority, &wireless_manager_task_handle);
    // xTaskCreate(wireless_heartbeat_task, "wireless heartbeat task", 1024, NULL, 6, NULL);
    // 开始广播
    send_broadcast_packet();
    return ESP_OK;
}
