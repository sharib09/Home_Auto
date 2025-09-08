/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"

#define TAG "BOARD"

struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },
    { LED_OFF, LED_OFF, LED_G, "green" },
    { LED_OFF, LED_OFF, LED_B, "blue"  },
};


struct _led_state control_state[6]={
		{ LED_OFF, LED_OFF, LED_R, "L1"   },
		{ LED_OFF, LED_OFF, LED_G, "L2" },
		{ LED_OFF, LED_OFF, LED_B, "L3"  },
		{ LED_OFF, LED_OFF, LED_V, "L4"   },
		{ LED_OFF, LED_OFF, LED_I, "F" },
		{ LED_OFF, LED_OFF, LED_C, "S"  },
};

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < 3; i++) {
        if (led_state[i].pin != pin) {
            continue;
        }
        if (onoff == led_state[i].previous) {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state[i].name, (onoff ? "on" : "off"));
            return;
        }
        gpio_set_level(pin, onoff);
        led_state[i].previous = onoff;
        return;
    }

    ESP_LOGE(TAG, "LED is not found!");
}

static void board_led_init(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(led_state[i].pin);
        gpio_set_direction(led_state[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(led_state[i].pin, LED_OFF);
        led_state[i].previous = LED_OFF;
    }
}
void control_state_operation(uint8_t pin, uint8_t onoff)
{
	 for (int i = 0; i < 6; i++) {
	        if (control_state[i].pin != pin) {
	            continue;
	        }
	        if (onoff == control_state[i].previous) {
	            ESP_LOGW(TAG, "led %s is already %s",
	                     control_state[i].name, (onoff ? "on" : "off"));
	            return;
	        }
	        //gpio_set_level(pin, onoff);
	        control_state[i].previous = onoff;

	        return;
	    }

	    ESP_LOGE(TAG, "LED is not found!");
}
static void control_state_init(void)
{
	for (int i = 0; i < 6; i++) {
	        gpio_reset_pin(control_state[i].pin);
	        gpio_set_direction(control_state[i].pin, GPIO_MODE_OUTPUT);
	        //gpio_set_level(led_state[i].pin, LED_OFF);
	       // led_state[i].previous = LED_OFF;
	    }
}

void board_init(void)
{
    board_led_init();
    control_state_init();
}
