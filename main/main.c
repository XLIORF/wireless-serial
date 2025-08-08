#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/task.h"
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
#include "driver/uart.h"
#include "wlcon.h"

#define UART_BUF_SIZE CONFIG_UART_BUF_SIZE
#define EX_UART_NUM UART_NUM_0
#define WIRELESS_RECV_QUEUE_SIZE CONFIG_WLCON_IO_QUEUE_SIZE
#define WIRELESS_SEND_QUEUE_SIZE CONFIG_WLCON_IO_QUEUE_SIZE
static char *TAG = "MAIN";

static xQueueHandle wlcon_send_queue = NULL,
                    wlcon_recv_queue = NULL;

void uart_rx_task(void *param)
{
    uint8_t *serial_data = NULL;
    size_t rx_len = 0;
    buf_len_t wireless_data;

    while (1)
    {
        vTaskDelay(5);
        if (xQueueReceive(wlcon_recv_queue, &wireless_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            // 有数据接收
            if (wireless_data.len > 0 && wireless_data.buf != NULL)
            {
                ESP_LOGD(__FUNCTION__, "recv [esp_now->serial]:%s", (char *)wireless_data.buf);
                uart_write_bytes(EX_UART_NUM, (const char *)wireless_data.buf, wireless_data.len);
                free(wireless_data.buf);
            }
        }
        // 判断用户是否输入
        esp_err_t err = uart_get_buffered_data_len(EX_UART_NUM, &rx_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(__FUNCTION__, "Error: %d", err);
            continue;
        }
        if (rx_len <= 0)
            continue;
        // 未连接但是有用户输入，则提示输入无效
        if (!wlcon_is_connected())
        {
            // 直接清除输入缓冲区
            uart_flush_input(EX_UART_NUM);
            uart_write_bytes(EX_UART_NUM, "未连接输入无效\r\n", strlen("未连接输入无效\r\n"));
            continue;
        }
        // 读取输入并转发
        serial_data = malloc(rx_len);
        int serial_rx_len = uart_read_bytes(EX_UART_NUM, serial_data, rx_len, 0);
        if (serial_rx_len > 0)
        {
            ESP_LOGD(__FUNCTION__, "send [serial->esp_now]:%s", (char *)serial_data);
            // 发送数据包
            buf_len_t send_data = {
                .len = serial_rx_len,
                .buf = serial_data,
                .flag = 0x01,
            };
            if (xQueueSend(wlcon_send_queue, &send_data, pdMS_TO_TICKS(10)) != pdTRUE)
            {
                ESP_LOGE(__FUNCTION__, "Failed to send data into wlcon_send_queue.");
            }
        }
    }
}

void app_main()
{
    // 初始化无线连接
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    ESP_ERROR_CHECK(wlcon_init());
    // 注册无线输入输出队列
    wlcon_send_queue = xQueueCreate(WIRELESS_SEND_QUEUE_SIZE, sizeof(buf_len_t));
    wlcon_recv_queue = xQueueCreate(WIRELESS_RECV_QUEUE_SIZE, sizeof(buf_len_t));
    if (wlcon_send_queue == NULL || wlcon_recv_queue == NULL)
    {
        ESP_LOGE(TAG, "Create queue fail");
        esp_restart();
    }
    wlcon_io_register(wlcon_send_queue, wlcon_recv_queue);

    // 初始化串口
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_driver_install(EX_UART_NUM, CONFIG_UART_BUF_SIZE * 2, CONFIG_UART_BUF_SIZE * 2, 0, NULL, 0);
    xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 4, NULL);

    // 删除自身任务
    vTaskDelete(NULL);
}
