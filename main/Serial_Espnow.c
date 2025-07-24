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
#define ACK_RETRY_MAX 3
static const char *TAG = "Serial_ESPNow";

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t target_mac[ESP_NOW_ETH_ALEN] = {0xff};
static xQueueHandle espnow_cb_queue;
static bool is_peer = false;
static RingbufHandle_t s2w_buf = NULL;
static RingbufHandle_t w2s_buf = NULL;
// 标志两个设备的连接状态和串口的工作状态
// static EventGroupHandle_t con_status_group;
// static const int CSEG_CONNECTED_BIT = BIT0;
// static const int CSEG_SERIAL_RUN_BIT = BIT1;

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
    if (xQueueSend(espnow_cb_queue, &evt, pdMS_TO_TICKS(1)) != pdTRUE)
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
    if (!is_peer)
        memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);

    /* 将接收事件发送到队列中 */
    if (xQueueSend(espnow_cb_queue, &evt, pdMS_TO_TICKS(1)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Func[espnow_recv_cb] Send queue fail");
    }
    free(recv_cb->data);
}

esp_err_t espnow_send_ack()
{
    // 分配发送包内存   包格式：espnow_package_t + 数据 + 数据长度
    uint8_t send_len = sizeof(espnow_package_t);
    espnow_package_t *send_buf = (espnow_package_t *)malloc(send_len);
    if (send_buf == NULL)
    {
        ESP_LOGD(TAG, "分配内存失败！\n");
        return ESP_FAIL;
    }

    send_buf->type = ESPNOW_DATA_ACK;

    // 计算CRC校验值，计算前需将crc字段置0
    send_buf->crc = 0;

    // 通过ESP-NOW发送数据包
    if (esp_now_send(target_mac, (const uint8_t *)send_buf, send_len) != ESP_OK)
    {
        ESP_LOGW(TAG, "发送失败！\n");
        free(send_buf);
        return ESP_FAIL;
    }

    // 释放分配的内存
    free(send_buf);
    return ESP_OK;
}

/**
 * @brief 通过ESP-NOW发送数据包
 *
 * @param buf 要发送的数据缓冲区指针
 * @param len 要发送的数据长度
 * @param is_broadcast 是否广播发送标志
 * @return esp_err_t ESP_OK表示发送成功，ESP_FAIL表示发送失败
 */
esp_err_t espnow_send_package(void *buf, uint8_t len, bool is_broadcast)
{
    // 检查输入参数有效性
    if (buf == NULL && len != 0)
    {
        ESP_LOGD(TAG, "传入参数有误！\n");
        return ESP_FAIL;
    }

    // 分配发送包内存   包格式：espnow_package_t + 数据 + 数据长度
    uint8_t send_len = sizeof(espnow_package_t) + len + 1;
    espnow_package_t *send_buf = (espnow_package_t *)malloc(send_len);
    if (send_buf == NULL)
    {
        ESP_LOGD(TAG, "分配内存失败！\n");
        return ESP_FAIL;
    }

    send_buf->type = is_broadcast == true ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    send_buf->payload[0] = len; // 设置数据长度
    memcpy(send_buf->payload + 1, buf, len);

    // 计算CRC校验值，计算前需将crc字段置0
    send_buf->crc = 0;
    send_buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)send_buf, send_len); // 解析时，应先把crc设置为0再计算；

    // 通过ESP-NOW发送数据包
    if (esp_now_send(is_broadcast == true ? broadcast_mac : target_mac, (const uint8_t *)send_buf, send_len) != ESP_OK)
    {
        ESP_LOGW(TAG, "发送失败！\n");
        free(send_buf);
        return ESP_FAIL;
    }

    // 释放分配的内存
    free(send_buf);
    return ESP_OK;
}

void espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    // bool is_broadcast = true;
    bool is_ack = true;
    uint8_t nack_retry = 0;
    while (xQueueReceive(espnow_cb_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
        case ESPNOW_SEND_CB:
        {
            // espnow_event_send_cb_t *info = &evt.info.send_cb;
            if (!is_peer)
            {
                // vTaskDelay(500);
                espnow_send_package("hello", 5, true);
                // ESP_LOGI(TAG, "Send [hello] to " MACSTR "", MAC2STR(info->mac_addr));
            }
            break;
        }
        case ESPNOW_RECV_CB:
        {
            // 取出数据包
            espnow_event_recv_cb_t *info = &evt.info.recv_cb;
            espnow_package_t *package = (espnow_package_t *)info->data;
            if (package == NULL)
            {
                ESP_LOGE(TAG, "Recv package is null");
                break;
            }
            // 连接判断
            if (!is_peer && esp_now_is_peer_exist(info->mac_addr) == false)
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
                peer->encrypt = true;
                memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                memcpy(peer->peer_addr, info->mac_addr, ESP_NOW_ETH_ALEN);
                ESP_ERROR_CHECK(esp_now_add_peer(peer));
                ESP_LOGI(TAG, "已连接");
                is_peer = true;
                free(peer);
                memcpy(target_mac, info->mac_addr, ESP_NOW_ETH_ALEN);
            }
            else if (package->type == ESPNOW_DATA_ACK)
            {
                is_ack = true;
                nack_retry = 0;
                ESP_LOGI(TAG, "收到ACK包");
            }
            else if (package->type == ESPNOW_DATA_UNICAST)
            {
                // 校验数据
                uint16_t crc_recv = package->crc;
                uint16_t crc_cal = 0;
                package->crc = 0;
                crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)package, info->len);
                if (crc_cal != crc_recv)
                {
                    ESP_LOGE(TAG, "crc check error!");
                    break;
                }
                // 解包
                uint8_t *payload = package->payload + 1;
                uint8_t len = package->payload[0];
                ESP_LOGI(TAG, "recv from:" MACSTR " len: %d, 类型:%d, 内容：%.*s.", MAC2STR(info->mac_addr), info->len, package->type, len, payload);
                if (is_peer)
                {
                    // TODO
                    espnow_send_ack();
                    if (xRingbufferSend(w2s_buf, payload, len, 0) != pdTRUE)
                    {
                        ESP_LOGE(TAG, "Send to w2s_buf fail");
                    }
                }
            }
            else
            {
            }
            break;
        }
        default:
            ESP_LOGE(TAG, "Callback type error: %d", evt.id);
            break;
        }
    }
}

void uart_rx_task(void *param)
{
    uint8_t data[RX_BUF_SIZE] = {0};
    size_t rx_len = 0;
    while (1)
    {
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
        if (!is_peer)
        {
            // 直接清除输入缓冲区
            uart_flush_input(EX_UART_NUM);
            uart_write_bytes(EX_UART_NUM, "未连接输入无效\r\n", strlen("未连接输入无效\r\n"));
            continue;
        }
        // 读取输入并转发
        int len = uart_read_bytes(EX_UART_NUM, data, RX_BUF_SIZE, 0);
        if (len > 0)
        {
            ESP_LOGD(TAG, "send [serial->esp_now]:%s", (char *)data);
            if (xRingbufferSend(s2w_buf, data, len, 0) != pdTRUE)
            {
                ESP_LOGE(TAG, "Send to s2w_buf fail");
            }
        }
    }
}

void send_task(void *param)
{
    size_t item_size;
    char *item;
    while (1)
    {
        if (!is_peer)
        {
            vTaskDelay(1000);
            continue;
        }

        item = (char *)xRingbufferReceiveUpTo(w2s_buf, &item_size, 0, BUF_SIZE);
        // Check received data
        if (item != NULL)
        {
            uart_write_bytes(EX_UART_NUM, item, item_size);
            // Return Item
            vRingbufferReturnItem(w2s_buf, (void *)item);
        }
        else
        {
            // Failed to receive item
            printf("Failed to receive1 item\n");
        }
        item = (char *)xRingbufferReceiveUpTo(s2w_buf, &item_size, 0, BUF_SIZE);
        // Check received data
        if (item != NULL)
        {
            espnow_send_package(item, item_size, false);
            // Return Item
            vRingbufferReturnItem(s2w_buf, (void *)item);
        }
        else
        {
            // Failed to receive item
            printf("Failed to receive2 item\n");
        }
    }
}

/**
 * @brief 初始化WiFi模块
 *
 * 该函数负责初始化WiFi模块，确保模块只被初始化一次。
 * 它会配置TCP/IP协议，设置事件循环，配置WiFi控制器和接口，
 * 并启动WiFi功能。此外，它还会设置WiFi工作模式和频道。
 */
void wifi_init(void)
{
    // 静态变量，用于标记WiFi模块是否已被初始化
    static bool initialized = false;

    // 如果WiFi模块已经被初始化，则直接返回，避免重复初始化
    if (initialized)
    {
        return;
    }

    // 初始化TCP/IP协议栈
    tcpip_adapter_init();

    // 创建默认的事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 使用默认配置初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 设置WiFi存储方式为RAM
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // 设置WiFi工作模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));

    // 启动WiFi模块
    ESP_ERROR_CHECK(esp_wifi_start());

    // 设置WiFi频道
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0));

    // 标记WiFi模块为已初始化
    initialized = true;
}

/**
 * Initialize ESP-NOW.
 *
 * This function initializes the ESP-NOW functionality, including creating a message queue, initializing ESP-NOW,
 * registering send and receive callback functions, setting the PMK, and adding broadcast peer information to the peer list.
 *
 * @return esp_err_t Returns the result of the initialization, ESP_OK for success, ESP_FAIL for failure.
 */
esp_err_t Serial_Espnow_init(void)
{
    // Create a queue for ESP-NOW events
    espnow_cb_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_cb_queue == NULL)
    {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }
    s2w_buf = xRingbufferCreate(BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    w2s_buf = xRingbufferCreate(BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (s2w_buf == NULL || w2s_buf == NULL)
    {
        ESP_LOGE(TAG, "Create ringbuffer fail"); 
        vQueueDelete(espnow_cb_queue);
        return ESP_FAIL;
    }

    // Initialize ESP-NOW and register send and receive callback functions
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    // Set the PMK for ESP-NOW encryption
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(espnow_cb_queue);
        esp_now_deinit();
        return ESP_FAIL;
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);
    uint32_t esp_now_version;
    esp_now_get_version(&esp_now_version);
    ESP_LOGI(TAG, "ESP-NOW Version: %d", esp_now_version);
    xTaskCreate(send_task, "send_task", 4096, NULL, 5, NULL);
    return ESP_OK;
}