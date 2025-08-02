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

static bool wait_ack = false;
// 是否作为主机
static bool is_master = false;
// 无线通信状态
wireless_status_t status = WIRELESS_STATUS_BROADCAST;
// ESP-NOW 发送队列
static xQueueHandle espnow_send_queue;
static xQueueHandle espnow_send_priority_queue;
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
static int espnow_send_task_proiority = CONFIG_PRORITY_ESPNOW_SNED;
static int wireless_manager_task_proiority = CONFIG_WLCON_MANAGER_PRORITY;
// 任务句柄
static TaskHandle_t espnow_send_task_handle = NULL;
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
static bool send_broadcast_packet()
{
    wireless_packet_t *packet = malloc(sizeof(wireless_packet_t) + 1);
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
    espnow_packet_t espnow_packet = {
        .len = sizeof(wireless_packet_t) + 1,
        .data = (uint8_t *)packet,
        .mac = broadcast_mac,
    };
    if (xQueueSend(espnow_send_queue, &espnow_packet, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Send broadcast packet to espnow_send_queue fail!");
        free(packet);
        return ESP_FAIL;
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
    espnow_packet_t espnow_packet = {
        .len = p_len,
        .data = (uint8_t *)packet,
        .mac = broadcast_mac,
    };
    if (xQueueSend(espnow_send_queue, &espnow_packet, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Send broadcast packet to espnow_send_queue fail!");
        free(packet);
        return ESP_FAIL;
    }

    return true;
}

static bool send_heartbeat_packet(int type)
{
    size_t p_len = sizeof(wireless_packet_t) + 1;
    wireless_packet_t *packet = malloc(p_len);
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "Malloc packet fail");
        return false;
    }
    packet->type = WIRELESS_PACKET_TYPE_HEARTBEAT;
    packet->length = 1;
    packet->version = WIRELESS_PACKET_VERSION;
    packet->crc = 0;
    packet->payload[0] = type;
    // packet->crc = crc16_le(UINT16_MAX, (uint8_t const *)packet, p_len);
    espnow_packet_t espnow_packet = {
        .len = p_len,
        .data = (uint8_t *)packet,
        .mac = broadcast_mac,
    };
    if (xQueueSend(espnow_send_priority_queue, &espnow_packet, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Send broadcast packet to espnow_send_queue fail!");
        free(packet);
        return ESP_FAIL;
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
    espnow_packet_t espnow_packet = {
        .len = p_len,
        .data = (uint8_t *)packet,
        .mac = broadcast_mac,
    };
    if (xQueueSend(espnow_send_queue, &espnow_packet, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Send broadcast packet to espnow_send_queue fail!");
        free(packet);
        return ESP_FAIL;
    }

    return true;
}

// 优化后的CRC校验宏
#define CRC_CHECK(packet) ({                                                                                \
    uint8_t crc_recv = packet->crc;                                                                         \
    packet->crc = 0;                                                                                        \
    uint8_t crc_calc = crc16_le(UINT16_MAX, (uint8_t *)packet, sizeof(wireless_packet_t) + packet->length); \
    packet->crc = crc_recv;                                                                                 \
    crc_calc == crc_recv;                                                                                   \
})

// 有高优先级的数据包需要发送时，提高发送任务的优先级
static void elevate_send_priority()
{
    // 获取优先队列的长度
    BaseType_t queue_length = uxQueueMessagesWaiting(espnow_send_priority_queue);
    // 如果优先队列中有数据，则提升发送任务的优先级
    if (queue_length > 0 && espnow_send_task_handle != NULL)
    {
        // 提升发送任务的优先级
        vTaskPrioritySet(espnow_send_task_handle, CONFIG_PRORITY_ESPNOW_SNED + 3);
        vTaskDelay(pdMS_TO_TICKS(100)); // 确保任务调度器有时间处理优先级变更
    }
    // 如果发送任务的优先级低于配置的优先级，则提升其优先级
    else if (espnow_send_task_handle != NULL)
    {
        BaseType_t current_priority = uxTaskPriorityGet(espnow_send_task_handle);
        if (current_priority != CONFIG_PRORITY_ESPNOW_SNED)
        {
            vTaskPrioritySet(espnow_send_task_handle, CONFIG_PRORITY_ESPNOW_SNED);
        }
    }
    // 如果发送队列中的数量超过配置最大值的三分之二，则提升优先级
    BaseType_t send_queue_length = uxQueueMessagesWaiting(espnow_send_queue);
    if (espnow_send_task_handle != NULL && send_queue_length > (ESPNOW_SEND_QUEUE_SIZE * 2 / 3))
    {
        BaseType_t current_priority = uxTaskPriorityGet(espnow_send_task_handle);
        if (current_priority < CONFIG_PRORITY_ESPNOW_SNED + 3)
        {
            vTaskPrioritySet(espnow_send_task_handle, CONFIG_PRORITY_ESPNOW_SNED + 3);
        }
    }
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
        elevate_send_priority();
        if (status == WIRELESS_STATUS_CONNECTED)
        {
            printf("距离上次心跳已过去%dms\n", heartbeat_time);
            if (heartbeat_time > (CONFIG_HEARTBEAT_INTERVAL + 2000))
            {
                // 心跳超时, 连接断开
                printf("心跳超时, 连接断开\n");
                printf("heartbeat_time: %d ms, %d ms\n", heartbeat_time, (CONFIG_HEARTBEAT_INTERVAL + 1500));
                status = WIRELESS_STATUS_DISCONNECTED;
            }
            else if (is_master && heartbeat_time > (CONFIG_HEARTBEAT_INTERVAL / 2))
            {
                // 发送心跳包
                if (heartbeat_interval > (CONFIG_HEARTBEAT_INTERVAL / 2))
                {
                    heartbeat_interval = 0; // 重置心跳发送间隔
                    send_heartbeat_packet(1);
                }
            }
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
        // 回调处理
        if (xQueueReceive(espnow_cb_queue, &evt, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            // 收到数据包
            if (evt.id == ESPNOW_RECV_CB)
            {
                ESP_LOGD(TAG, "收到数据包，类型: %u", ((wireless_packet_t *)(evt.info.recv_cb.data))->type);
                // 有效检测
                wireless_packet_t *packet = (wireless_packet_t *)evt.info.recv_cb.data;
                if (sizeof(wireless_packet_t) > evt.info.recv_cb.len || packet == NULL)
                {
                    ESP_LOGW(TAG, "收到的无线数据包异常！");
                    continue;
                }
                // 版本检测
                if (packet->version != WIRELESS_PACKET_VERSION)
                {
                    // 不支持的包直接抛弃
                    ESP_LOGW(TAG, "不支持的无线数据包版本！");
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
                case WIRELESS_PACKET_TYPE_BROADCAST:
                {
                    if (status != WIRELESS_STATUS_BROADCAST && status != WIRELESS_STATUS_CONNECT_RST)
                    {
                        ESP_LOGD(TAG, "收到广播包，但当前状态不是广播状态，丢弃数据包");
                        break;
                    }
                    if (retry_count >= 3)
                    {
                        // 连接失败, 继续广播
                        status = WIRELESS_STATUS_BROADCAST;
                        retry_count = 0;
                        is_master = false;
                    }
                    else if (master_ruling_code > packet->payload[0])
                    {
                        printf("作为主机端\n");
                        is_master = true;
                        if (esp_now_is_peer_exist(evt.info.recv_cb.mac_addr) == false)
                        {
                            // 如果是广播包，且未连接，则添加对方为peer
                            esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                            if (peer == NULL)
                            {
                                ESP_LOGE(TAG, "Malloc peer information fail");
                                vTaskDelete(NULL);
                            }
                            memset(peer, 0, sizeof(esp_now_peer_info_t));
                            peer->channel = CONFIG_ESPNOW_CHANNEL;
                            peer->ifidx = ESPNOW_WIFI_IF;
                            peer->encrypt = true;
                            memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                            memcpy(peer->peer_addr, evt.info.recv_cb.mac_addr, ESP_NOW_ETH_ALEN);
                            ESP_ERROR_CHECK(esp_now_add_peer(peer));
                            free(peer);
                            // 记录对方MAC地址
                            memcpy(target_mac, evt.info.recv_cb.mac_addr, ESP_NOW_ETH_ALEN);
                        }
                        // 停止广播
                        status = WIRELESS_STATUS_CONNECT_RST;
                        if (xTaskGetTickCount() - last_connect_rst_time < pdMS_TO_TICKS(CONFIG_CONNECT_INTERVAL))
                        {
                            continue;
                        }

                        connect_code = esp_random() & 0xff;
                        send_connect_packet(1, connect_code);
                        ESP_LOGD(TAG, "发送连接请求包，连接码: %d", connect_code);
                        last_connect_rst_time = xTaskGetTickCount();
                        retry_count++;
                    }
                    break;
                }
                case WIRELESS_PACKET_TYPE_CONNECT:
                {
                    if (status == WIRELESS_STATUS_CONNECTED)
                    {
                        break; // 如果已经在连接状态，则忽略连接请求包
                    }
                    if (esp_now_is_peer_exist(evt.info.recv_cb.mac_addr) == false)
                    {
                        // 如果是广播包，且未连接，则添加对方为peer
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL)
                        {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, evt.info.recv_cb.mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK(esp_now_add_peer(peer));
                        free(peer);
                        // 记录对方MAC地址
                        memcpy(target_mac, evt.info.recv_cb.mac_addr, ESP_NOW_ETH_ALEN);
                    }
                    // crc 校验
                    if (CRC_CHECK(packet) == false)
                    {
                        ESP_LOGE(TAG, "crc check failed!");
                        continue;
                    }
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
                            heartbeat_time = 0;
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
                            heartbeat_time = 0;
                            printf("连接建立成功\n");
                        }
                        else
                        {
                            printf("连接建立包校验码错误\n");
                            status = WIRELESS_STATUS_BROADCAST;
                        }
                    }
                    break;
                }
                case WIRELESS_PACKET_TYPE_HEARTBEAT: // 心跳包
                {
                    heartbeat_time = 0;
                    ESP_LOGD(TAG, "收到心跳包，类型: %d", packet->payload[0]);
                    // if (CRC_CHECK(packet) == false)
                    // {
                    //     ESP_LOGW(TAG, "crc check failed!");
                    //     break;
                    // }
                    if (packet->payload[0] == 0x01) // 请求包
                    {
                        send_heartbeat_packet(2);
                    }
                    else if (packet->payload[0] == 0x02) // 应答包
                    {
                        // heartbeat_time = 0;
                    }
                    break;
                }
                case WIRELESS_PACKET_TYPE_DATA:
                {
                    // crc  校验
                    if (CRC_CHECK(packet) == false)
                    {
                        ESP_LOGD(TAG, "crc check failed!");
                        break;
                    }
                    heartbeat_time = 0;

                    // 丢进输出队列中
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
                }
                case WIRELESS_PACKET_TYPE_DATA_ACK:
                {
                    ESP_LOGD(TAG, "收到数据包应答");
                    // TODO 数据发送成功，从队列中移除
                    wait_ack = false;
                    heartbeat_time = 0;
                    break;
                }
                default:
                {
                    ESP_LOGW(TAG, "未知数据包类型！");
                }
                }
                free(packet);
            }
            else if (evt.id == ESPNOW_SEND_CB)
            {
            }
            else
            {
                ESP_LOGE(TAG, "错误的回调类型！");
            }
        }
    }
}

void esp_now_task(void *pvParameters)
{
    esp_now_init();
    espnow_packet_t esp_now_data;
    while (1)
    {
        // 短的超时时间应该可以避免单个队列长时间阻塞和防止狗叫
        if (xQueueReceive(espnow_send_priority_queue, &esp_now_data, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            esp_now_send(esp_now_data.mac, esp_now_data.data, esp_now_data.len);
            free(esp_now_data.data);
        }
        if (xQueueReceive(espnow_send_queue, &esp_now_data, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            if (!wait_ack)
            {
                esp_now_send(esp_now_data.mac, esp_now_data.data, esp_now_data.len);
                free(esp_now_data.data);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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
            uint16_t len = sizeof(wireless_packet_t) + serial_rx_len;
            espnow_packet_t espnow_packet = {
                .len = len,
                .data = malloc(len),
                .mac = target_mac,
            };
            wireless_packet_t *wp = (wireless_packet_t *)espnow_packet.data;
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
            if (xQueueSend(espnow_send_queue, &espnow_packet, pdMS_TO_TICKS(10)) != pdTRUE)
            {
                ESP_LOGW(TAG, "Send data packet to espnow_send_queue fail!");
            }
        }
    }
}

void wireless_heartbeat_handler(void *pvParameters)
{
    heartbeat_time += 10;
    heartbeat_interval += 10;
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
    espnow_send_queue = xQueueCreate(ESPNOW_SEND_QUEUE_SIZE, sizeof(espnow_packet_t));
    espnow_send_priority_queue = xQueueCreate(ESPNOW_SEND_PRORITY_QUEUE_SIZE, sizeof(espnow_packet_t));
    wireless_recv_queue = xQueueCreate(WIRELESS_RECV_QUEUE_SIZE, sizeof(buf_len_t));
    if (espnow_cb_queue == NULL || espnow_send_queue == NULL ||
        espnow_send_priority_queue == NULL || wireless_recv_queue == NULL)
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
    // esp_timer_handle_t heartbeat_timer;
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
    xTaskCreate(esp_now_task, "esp_now_task", 4096, NULL, espnow_send_task_proiority, &espnow_send_task_handle);

    // 开始广播
    send_broadcast_packet();
    // esp_now_send(broadcast_mac, (uint8_t *)packet, sizeof(wireless_packet_t)+1);
    return ESP_OK;
}
