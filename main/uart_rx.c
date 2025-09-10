#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_vfs_dev.h"

#include "uart_if.h"

/* Use the default console UART0 */
#define EX_UART_NUM       UART_NUM_0
#define BUF_SIZE          (1024)
#define UART_TASK_STACK   (4096)
#define UART_TASK_PRIO    (12)

static const char *TAG = "uart_rx";
static QueueHandle_t uart0_queue = NULL;

/* Globals defined in main.c */
extern volatile uint8_t ReceivedByte;
extern volatile bool    uart_recieve_flag;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *dtmp = (uint8_t*) malloc(BUF_SIZE);
    if (!dtmp) {
        ESP_LOGE(TAG, "malloc failed for RX buffer");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        if (uart0_queue == NULL) {
            ESP_LOGE(TAG, "uart0_queue is NULL, exiting task");
            break;
        }

        if (xQueueReceive(uart0_queue, &event, pdMS_TO_TICKS(1000)) == pdTRUE) {
            switch (event.type) {
                case UART_DATA: {
                    int to_read = event.size;
                    if (to_read > BUF_SIZE) to_read = BUF_SIZE;

                    int rlen = uart_read_bytes(EX_UART_NUM, dtmp, to_read, pdMS_TO_TICKS(20));
                    if (rlen > 0) {
                        /* Print in ESP_LOG format: ASCII + HEX */
                        ESP_LOGI(TAG, "Received %d bytes", rlen);

                        // Print ASCII safely
                        char ascii_buf[65]; // up to 64 chars + null
                        int copy_len = (rlen < 64) ? rlen : 64;
                        for (int i = 0; i < copy_len; i++) {
                            ascii_buf[i] = (dtmp[i] >= 32 && dtmp[i] <= 126) ? dtmp[i] : '.';
                        }
                        ascii_buf[copy_len] = '\0';
                        ESP_LOGI(TAG, "ASCII: %s", ascii_buf);

                        // Print HEX
                        char hex_buf[3 * 32 + 1]; // 32 bytes max per line
                        int idx = 0;
                        for (int i = 0; i < rlen && i < 32; i++) {
                            idx += snprintf(&hex_buf[idx], sizeof(hex_buf) - idx, "%02X ", dtmp[i]);
                        }
                        hex_buf[idx] = '\0';
                        ESP_LOGI(TAG, "HEX: %s%s", hex_buf, (rlen > 32) ? "..." : "");

                        /* Update globals with the last byte */
                        ReceivedByte = dtmp[rlen - 1];
                        uart_recieve_flag = true;
                    }
                    break;
                }

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART HW FIFO overflow");
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART ring buffer full");
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                case UART_BREAK:
                    ESP_LOGW(TAG, "UART RX break");
                    break;

                case UART_PARITY_ERR:
                    ESP_LOGW(TAG, "UART parity error");
                    break;

                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "UART frame error");
                    break;

                default:
                    ESP_LOGW(TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }

    free(dtmp);
    vTaskDelete(NULL);
}

void uart_rx_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
        .source_clk = UART_SCLK_DEFAULT
#endif
    };

    esp_err_t err = uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    if (err != ESP_OK || uart0_queue == NULL) {
        ESP_LOGE(TAG, "uart_driver_install failed (%d)", (int)err);
        return;
    }

    err = uart_param_config(EX_UART_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed (%d)", (int)err);
        return;
    }

    uart_set_pin(EX_UART_NUM,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

    esp_vfs_dev_uart_use_driver(EX_UART_NUM);
}

void uart_rx_start(void)
{
    if (uart0_queue == NULL) {
        ESP_LOGE(TAG, "uart_rx_start called without a valid queue. Did uart_rx_init() succeed?");
        return;
    }
    xTaskCreate(uart_event_task, "uart_event_task", UART_TASK_STACK, NULL, UART_TASK_PRIO, NULL);
}

/* Optional helpers */
void uart2_smoketest_init(void)  {}
void uart2_smoketest_start(void) {}
int  uart_tx_write(const uint8_t *data, int len)
{
    if (!data || len <= 0) return 0;
    return uart_write_bytes(EX_UART_NUM, (const char *)data, len);
}
