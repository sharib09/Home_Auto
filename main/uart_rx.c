#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_vfs_dev.h"   // for esp_vfs_dev_uart_use_driver

#include "uart_if.h"

/* Use the default console UART0 */
#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE    (1024)
#define UART_TASK_STACK (4096)
#define UART_TASK_PRIO  (12)

static const char *TAG = "uart_rx";
static QueueHandle_t uart0_queue = NULL;

/* These are defined in main.c and declared in uart_if.h */
extern volatile uint8_t ReceivedByte;
extern volatile bool    uart_recieve_flag;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *dtmp = (uint8_t*) malloc(BUF_SIZE);
    if (!dtmp) {
        ESP_LOGE(TAG, "malloc failed");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        if (uart0_queue == NULL) {
            ESP_LOGE(TAG, "uart0_queue is NULL, exiting task");
            break;
        }

        // Wait up to 1000 ms to avoid WDT if something goes wrong
        if (xQueueReceive(uart0_queue, &event, pdMS_TO_TICKS(1000)) == pdTRUE) {
            switch (event.type) {
                case UART_DATA: {
                    // Read exactly what arrived (cap at 1 byte if that's your protocol)
                    // You can also read event.size bytes for bursty input
                    int r = uart_read_bytes(EX_UART_NUM, (uint8_t *)&ReceivedByte, 1, pdMS_TO_TICKS(20));
                    if (r > 0) {
                        uart_recieve_flag = true;
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    // Avoid spamming logs from within the event loop
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                case UART_BUFFER_FULL:
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                case UART_BREAK:
                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                default:
                    // Keep logging minimal; logs themselves also generate UART0 traffic
                    break;
            }
        }
        // else: timeout tick; loop again
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

    // Install driver FIRST
    esp_err_t err = uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    if (err != ESP_OK || uart0_queue == NULL) {
        ESP_LOGE(TAG, "uart_driver_install failed (%d), queue=%p", (int)err, (void*)uart0_queue);
        return; // Do NOT start task if driver/queue failed
    }

    // Configure parameters
    err = uart_param_config(EX_UART_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed (%d)", (int)err);
        return;
    }

    // On UART0, keep default pins to avoid breaking console routing
    uart_set_pin(EX_UART_NUM,
                 UART_PIN_NO_CHANGE,  /* TX (GPIO1) */
                 UART_PIN_NO_CHANGE,  /* RX (GPIO3) */
                 UART_PIN_NO_CHANGE,  /* RTS */
                 UART_PIN_NO_CHANGE); /* CTS */

    // IMPORTANT for UART0: route console I/O through the installed driver
    // This prevents conflicts between console and your driver usage.
    esp_vfs_dev_uart_use_driver(EX_UART_NUM);
}

void uart_rx_start(void)
{
    if (uart0_queue == NULL) {
        // init() probably failed — avoid starting the task and rebooting on assert
        ESP_LOGE(TAG, "uart_rx_start called without a valid queue. Did uart_rx_init() succeed?");
        return;
    }
    xTaskCreate(uart_event_task, "uart_event_task", UART_TASK_STACK, NULL, UART_TASK_PRIO, NULL);
}

/* Optional stubs to satisfy header if you don't use them yet */
void uart2_smoketest_init(void)  {}
void uart2_smoketest_start(void) {}

int uart_tx_write(const uint8_t *data, int len)
{
    if (!data || len <= 0) return 0;
    return uart_write_bytes(EX_UART_NUM, (const char *)data, len);
}
