#pragma once
#include "driver/gpio.h"

/* 
 * CENTRAL PLACE FOR YOUR BOARD PIN MAP
 * Replace these GPIO_NUM_* values with your actual wiring.
 * The #ifndef guards let you override them in sdkconfig or other headers if needed.
 */

#ifndef RELAY1
#define RELAY1     GPIO_NUM_12
#endif
#ifndef RELAY2
#define RELAY2     GPIO_NUM_13
#endif
#ifndef RELAY3
#define RELAY3     GPIO_NUM_14
#endif
#ifndef RELAY4
#define RELAY4     GPIO_NUM_27
#endif
#ifndef RELAY5
#define RELAY5     GPIO_NUM_25
#endif
#ifndef RELAY6
#define RELAY6     GPIO_NUM_26
#endif
#ifndef RELAY7
#define RELAY7     GPIO_NUM_33
#endif
#ifndef RELAY8
#define RELAY8     GPIO_NUM_32
#endif
#ifndef RELAY9
#define RELAY9     GPIO_NUM_4
#endif

#ifndef DATA_PIN
#define DATA_PIN   GPIO_NUM_18    /* shift-reg data */
#endif
#ifndef CLOCK_PIN
#define CLOCK_PIN  GPIO_NUM_19    /* shift-reg clock */
#endif
#ifndef LATCH_PIN
#define LATCH_PIN  GPIO_NUM_21    /* shift-reg latch */
#endif

/* If you use active-low relays, add helpers here:
   #define RELAY_ON  0
   #define RELAY_OFF 1
*/
