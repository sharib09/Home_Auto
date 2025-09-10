#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "uart_if.h"     // for ReceivedByte / uart_recieve_flag
#include "system_task.h"
#include "pins.h" 

/* ===== Config ===== */
#define SYSTEM_TASK_STACK   (4096)
#define SYSTEM_TASK_PRIO    (11)

static const char *TAG = "SystemTask";

/* ===== Extern app state (provided elsewhere in your project) =====
   These are used by your original logic. Declare them here; define them once
   in whichever module owns the global state (e.g., main.c).
*/
extern volatile bool L1, L2, L3, L4, F, S;
extern volatile bool FS1, FS2, FS3;

/* If your RELAYx / DATA_PIN / CLOCK_PIN / LATCH_PIN are macros (#define),
   no externs are needed here. Keep your original header that defines them. */

/* ===== UART shared state from uart_if.h (already declared there) ===== */
extern volatile uint8_t ReceivedByte;
extern volatile bool    uart_recieve_flag;

/* ===== Local "last state" latches (keep inside this task) ===== */
static bool last_L1_State = 0;
static bool last_L2_State = 0;
static bool last_L3_State = 0;
static bool last_L4_State = 0;
static bool last_F_State  = 0;
static bool last_F1_State = 0;
static bool last_F2_State = 0;
static bool last_F3_State = 0;
static bool last_S_State  = 0;

void system_task_init(void)
{
    /* Configure GPIO directions (your original code) */
    gpio_set_direction(RELAY1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY2, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY3, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY4, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY5, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY6, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY7, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY8, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY9, GPIO_MODE_OUTPUT);

    gpio_set_direction(DATA_PIN,  GPIO_MODE_OUTPUT);
    gpio_set_direction(CLOCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LATCH_PIN, GPIO_MODE_OUTPUT);

    /* Optionally set initial output levels here, if desired */
}

static void System_operation_task(void *arg)
{
    ESP_LOGI(TAG, "System operation task started");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));

        /* ========== UART byte hook (prints when a new byte arrives) ========== */
        if (uart_recieve_flag) {
            uart_recieve_flag = false;                 // clear first
            uint8_t b = ReceivedByte;                  // snapshot
            printf("RX byte: %u (0x%02X) '%c'\n",
                   (unsigned)b, (unsigned)b,
                   (b >= 32 && b <= 126) ? (char)b : '.');
            /* TODO: if needed, decode 'b' to change L1/L2/... here */
        }

        /* ========== Your original relay/shift logic ========== */

        /* L1..L4 */
        bool L1_State = L1;
        if (L1_State != last_L1_State) {
            last_L1_State = L1_State;
            gpio_set_level(RELAY1, L1_State);
        }

        bool L2_State = L2;
        if (L2_State != last_L2_State) {
            last_L2_State = L2_State;
            gpio_set_level(RELAY2, L2_State);
        }

        bool L3_State = L3;
        if (L3_State != last_L3_State) {
            last_L3_State = L3_State;
            gpio_set_level(RELAY3, L3_State);
        }

        bool L4_State = L4;
        if (L4_State != last_L4_State) {
            last_L4_State = L4_State;
            gpio_set_level(RELAY4, L4_State);
        }

        /* F */
        bool F_State = F;
        if (F_State != last_F_State) {
            last_F_State = F_State;
            // gpio_set_level(RELAY1, F_State);  // (your original comment)
        }

        /* Shift register (shift_Out call was commented) */
        uint8_t shift_value =
            (L1 << 7) | (L2 << 6) | (L3 << 5) | (L4 << 4) |
            (F  << 3) | (S  << 2) | (0  << 1) | (0  << 0);

        gpio_set_level(LATCH_PIN, 0);
        // shift_Out(DATA_PIN, CLOCK_PIN, LSBFIRST, shift_value);
        gpio_set_level(LATCH_PIN, 1);

        /* Fan speeds FS1/FS2/FS3 */
        bool F1_State = FS1;
        bool F2_State = FS2;
        bool F3_State = FS3;

        if (F1_State != last_F1_State || F2_State != last_F2_State || F3_State != last_F3_State) {
            last_F1_State = F1_State;
            last_F2_State = F2_State;
            last_F3_State = F3_State;

            if ((F3_State == 0) && (FS2 == 0) && (F1_State == 1) && (F == 1)) {
                printf("FAN SPEED 1\n");
                gpio_set_level(RELAY5, 1);
                gpio_set_level(RELAY6, 0);
                gpio_set_level(RELAY7, 0);
                gpio_set_level(RELAY8, 0);
                gpio_set_level(5,  1);
                gpio_set_level(32, 0);
                gpio_set_level(33, 0);

            } else if ((F3_State == 0) && (FS2 == 1) && (F1_State == 0) && (F == 1)) {
                printf("FAN SPEED 2\n");
                gpio_set_level(RELAY5, 1);
                gpio_set_level(RELAY6, 1);
                gpio_set_level(RELAY7, 0);
                gpio_set_level(RELAY8, 0);
                gpio_set_level(32, 1);
                gpio_set_level(33, 0);
                gpio_set_level(5,  0);

            } else if ((F3_State == 0) && (FS2 == 1) && (F1_State == 1) && (F == 1)) {
                printf("FAN SPEED 3\n");
                gpio_set_level(RELAY5, 0);
                gpio_set_level(RELAY6, 1);
                gpio_set_level(RELAY7, 1);
                gpio_set_level(RELAY8, 0);
                gpio_set_level(33, 1);
                gpio_set_level(32, 0);
                gpio_set_level(5,  0);

            } else if ((F3_State == 1) && (FS2 == 0) && (F1_State == 0) && (F == 1)) {
                printf("FAN SPEED 4\n");
                gpio_set_level(RELAY5, 1);
                gpio_set_level(RELAY6, 1);
                gpio_set_level(RELAY7, 1);
                gpio_set_level(RELAY8, 0);
                gpio_set_level(5,  1);
                gpio_set_level(32, 1);
                gpio_set_level(33, 0);

            } else if ((F3_State == 1) && (FS2 == 0) && (F1_State == 1) && (F == 1)) {
                printf("FAN SPEED 5\n");
                gpio_set_level(RELAY5, 0);
                gpio_set_level(RELAY6, 0);
                gpio_set_level(RELAY7, 0);
                gpio_set_level(RELAY8, 1);
                gpio_set_level(33, 1);
                gpio_set_level(5,  1);
                gpio_set_level(32, 0);

            } else {
                gpio_set_level(RELAY5, 0);
                gpio_set_level(RELAY6, 0);
                gpio_set_level(RELAY7, 0);
                gpio_set_level(RELAY8, 0);
                gpio_set_level(33, 0);
                gpio_set_level(5,  0);
                gpio_set_level(32, 0);
            }
        }

        /* S */
        bool S_State = S;
        if (S_State != last_S_State) {
            last_S_State = S_State;
            gpio_set_level(RELAY9, S_State);
        }

        /* Extra small delay like your original tail delay */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void system_task_start(void)
{
    xTaskCreate(System_operation_task, "system_operation_task",
                SYSTEM_TASK_STACK, NULL, SYSTEM_TASK_PRIO, NULL);
}
