#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
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
#include "espnow.h"
#include "driver/uart.h"

static const char *TAG = "LX_espnow";

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t target_mac[ESP_NOW_ETH_ALEN] = {0xff};
static xQueueHandle espnow_queue;
static bool is_peer = false;

void wifi_init(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0) );
}


static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memset(recv_cb->data, 0 ,len);
    memcpy(recv_cb->data, data, len);
    recv_cb->len = len;
    if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

esp_err_t espnow_init(void)
{
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }
    
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);
    
    return ESP_OK;
}

esp_err_t espnow_send_package(void *buf, uint8_t len, bool is_broadcast)
{
    if(buf == NULL && len != 0)
    {
        ESP_LOGD(TAG, "传入参数有误！\n");
        return ESP_FAIL;
    }
    uint8_t send_len = sizeof(espnow_package_t) + len;
    espnow_package_t *send_buf = (espnow_package_t *)malloc(send_len);
    if(send_buf == NULL)
    {
        ESP_LOGD(TAG, "分配内存失败！\n");
        return ESP_FAIL;
        
    }
    memset(send_buf, 0, send_len);
    send_buf->type = is_broadcast == true ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    memcpy(send_buf->payload, buf, len);
    send_buf->crc = 0;
    send_buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)send_buf, send_len); //解析时，应先把crc设置为0再计算；
    if(esp_now_send(is_broadcast == true?broadcast_mac:target_mac, (const uint8_t *)send_buf, send_len) != ESP_OK)
    // if(esp_now_send(is_broadcast == true?broadcast_mac:target_mac, (const uint8_t *)"hello from espnow", 17) != ESP_OK)
    {
        ESP_LOGD(TAG, "发送失败！\n");
        free(send_buf);
        return ESP_FAIL;
    }
    free(send_buf);
    return ESP_OK;
}

void espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    // bool is_broadcast = true;
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t * send_info = &evt.info.send_cb;
                // is_broadcast = IS_BROADCAST_ADDR(send_info->mac_addr);
                if(!is_peer)
                {
                    vTaskDelay(500);
                    espnow_send_package("hello", 5, true);
                    ESP_LOGI(TAG,"Send to "MACSTR"",MAC2STR(send_info->mac_addr));
                }
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_info = &evt.info.recv_cb;
                espnow_package_t *data = (espnow_package_t *)recv_info->data;
                uint16_t crc_recv = data->crc;
                uint16_t crc_cal = 0;
                data->crc = 0;
                crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)data, recv_info->len);
                // ESP_LOGI(TAG,"recv from:"MACSTR" len: %d, 校验结果：%d,类型:%d", MAC2STR(recv_info->mac_addr), recv_info->len, crc_cal == crc_recv, data->type);
                if(crc_cal != crc_recv)
                {
                    ESP_LOGI(TAG,"crc校验失败！");
                    break;
                }
                for(int i=0;i<recv_info->len;i++)
                {
                    printf("%c",recv_info->data[i]);
                }
                printf("\n");
                
                if (!is_peer && esp_now_is_peer_exist(recv_info->mac_addr) == false) {
                    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                    if (peer == NULL) {
                        ESP_LOGE(TAG, "Malloc peer information fail");
                        free(peer);
                        vTaskDelete(NULL);
                    }
                    memset(peer, 0, sizeof(esp_now_peer_info_t));
                    peer->channel = CONFIG_ESPNOW_CHANNEL;
                    peer->ifidx = ESPNOW_WIFI_IF;
                    peer->encrypt = true;
                    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                    memcpy(peer->peer_addr, recv_info->mac_addr, ESP_NOW_ETH_ALEN);
                    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                    is_peer = true;
                    ESP_LOGI(TAG,"已连接");
                    free(peer);
                    memcpy(target_mac, recv_info->mac_addr, ESP_NOW_ETH_ALEN);
                }

                if(data->type == ESPNOW_DATA_BROADCAST)
                {
                    vTaskDelay(500);
                    espnow_send_package("hello", 5, false);
                    ESP_LOGI(TAG,"Send to "MACSTR"",MAC2STR(recv_info->mac_addr));
                }
                else if(data->type == ESPNOW_DATA_UNICAST)
                {
                    uart_write_bytes(EX_UART_NUM, (const char *)data->payload, recv_info->len - sizeof(espnow_package_t));
                    // printf("printf:\n");
                    // for(int i=0;i<recv_info->len - sizeof(espnow_package_t);i++)
                    //     printf("%c",(char )(data->payload[i]));
                    // printf("\n");
                    // ESP_LOGI(TAG, "recv :%s",data->payload);
                }
                else
                {

                }
                free(data);
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
    uint8_t data[BUF_SIZE] = {0};
    memset(data, '\0', BUF_SIZE);
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // uart_write_bytes(EX_UART_NUM,(const char *) data,len);
        if(len > 0)
        {
            ESP_LOGI(TAG,"send :%s", (char *)data);
            espnow_send_package(data, len, false);
        }
        memset(data, '\0', BUF_SIZE);
    }
}
