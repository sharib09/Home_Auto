/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "strings.h"
#include "esp_system.h"
#include <stdlib.h>
#include "freertos/task.h"

#include "uart_if.h"

// void System_operation_task(void *arg);
// static void rx_task(void *arg);

// void init(void);



#define LIGHT

extern struct _led_state led_state[3];
extern struct _led_state control_state[6];
struct _led_state *control=NULL;

bool last_F1_State=0;
bool last_F2_State=0;

bool last_F3_State=0;
bool last_F_State=0;
bool last_L1_State=0;
bool last_L2_State=0;
bool last_L3_State=0;
bool last_L4_State=0;
bool last_S_State=0;

bool L1;
bool L2;
bool L3;
bool L4;
bool F;
bool FS1;
bool FS2;
bool FS3;
bool S;

#define RELAY1 19
#define RELAY2 19
#define RELAY3 19
#define RELAY4 19
#define RELAY5 19
#define RELAY6 19
#define RELAY7 19
#define RELAY8 19
#define RELAY9 19
#define DATA_PIN 19
#define CLOCK_PIN 19
#define LATCH_PIN 19
#define LSBFIRST 00000001

unsigned char control_data = 0;

unsigned char recieved_control_data = 0;

volatile bool    uart_recieve_flag = false;
volatile uint8_t ReceivedByte      = 0;

//static const int RX_BUF_SIZE = 1024;
#define TXD_PIN 1
#define RXD_PIN 3

#define TAG "EXAMPLE"

#define CID_ESP 0x02E5

extern struct _led_state led_state[3];

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_0, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_0 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_1, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_1 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_2, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_2 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_3, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_3 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
};
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub,2+3,ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server= {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    
    },
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_0, &onoff_server_0),
};

static esp_ble_mesh_model_t extend_model_0[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_1, &onoff_server_1),
};

static esp_ble_mesh_model_t extend_model_1[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_2, &onoff_server_2),
};

static esp_ble_mesh_model_t extend_model_2[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_3, &onoff_server_3),
};

static esp_ble_mesh_model_t extend_model_3[] = {
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub, &level_server),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_0, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_1, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_2, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_3, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

/* Disable OOB security for SILabs Android app */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
#if 0
    .output_size = 4,
    .output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
    .input_size = 4,
    .input_actions = ESP_BLE_MESH_PUSH,
#else
    .output_size = 0,
    .output_actions = 0,
#endif
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
    board_led_operation(LED_G, LED_OFF);
}

static void example_change_led_state(esp_ble_mesh_model_t *model,
                                     esp_ble_mesh_msg_ctx_t *ctx, uint8_t onoff)
{
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    struct _led_state *led = NULL;
    uint8_t i=0;;

    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst)) {
        for (i = 0; i < elem_count; i++) {
            if (ctx->recv_dst == (primary_addr + i)) {
                led = &led_state[i];
                //board_led_operation(led->pin, onoff);
                   control=&control_state[i];
               // board_led_operation(led->pin, onoff);
                control_state_operation(control->pin, onoff);
                L1=control_state[0].previous;
                L2=control_state[1].previous;
                L3=control_state[2].previous;
                L4=control_state[3].previous;
             //   F=control_state[4].previous;
             //   S=control_state[5].previous;
                control_data=(L1<<7)|(L2<<6)|(L3<<5)|(L4<<4)|(F<<3)|(FS3<<2)|(FS2<<1)|(FS1<<0);
                ESP_LOGI(TAG, "L1_state 0x%02x", L1);
                ESP_LOGI(TAG, "L2_state 0x%02x", L2);
                ESP_LOGI(TAG, "L3_state 0x%02x", L3);
                ESP_LOGI(TAG, "L4_state 0x%02x", L4);
                               
            }
        }
    } else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst)) {
            led = &led_state[model->element->element_addr - primary_addr];
            //board_led_operation(led->pin, onoff);
               control=&control_state[i];
               // board_led_operation(led->pin, onoff);
                control_state_operation(control->pin, onoff);
                L1=control_state[0].previous;
                L2=control_state[1].previous;
                L3=control_state[2].previous;
                L4=control_state[3].previous;
             //   F=control_state[4].previous;
              //  S=control_state[5].previous;
                control_data=(L1<<7)|(L2<<6)|(L3<<5)|(L4<<4)|(F<<3)|(FS3<<2)|(FS2<<1)|(FS1<<0);
        }
    } else if (ctx->recv_dst == 0xFFFF) {
       // led = &led_state[model->element->element_addr - primary_addr];
       // board_led_operation(led->pin, onoff);
           control=&control_state[i];
               // board_led_operation(led->pin, onoff);
                control_state_operation(control->pin, onoff);
                L1=control_state[0].previous;
                L2=control_state[1].previous;
                L3=control_state[2].previous;
                L4=control_state[3].previous;
             //   F=control_state[4].previous;
              //  S=control_state[5].previous;
                control_data=(L1<<7)|(L2<<6)|(L3<<5)|(L4<<4)|(F<<3)|(FS3<<2)|(FS2<<1)|(FS1<<0);
    }
}

static void example_handle_gen_onoff_msg(esp_ble_mesh_model_t *model,
                                         esp_ble_mesh_msg_ctx_t *ctx,
                                         esp_ble_mesh_server_recv_gen_onoff_set_t *set)
{
    esp_ble_mesh_gen_onoff_srv_t *srv = (esp_ble_mesh_gen_onoff_srv_t *)model->user_data;

    switch (ctx->recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        esp_ble_mesh_server_model_send_msg(model, ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        break;
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
        if (set->op_en == false) {
            srv->state.onoff = set->onoff;
        } else {
            /* TODO: Delay and state transition */
            srv->state.onoff = set->onoff;
        }
        if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            esp_ble_mesh_server_model_send_msg(model, ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        }
        esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
            sizeof(srv->state.onoff), &srv->state.onoff, ROLE_NODE);
        example_change_led_state(model, ctx, srv->state.onoff);
        break;
    default:
        break;
    }
}

static inline void put_le16(uint8_t *dst, int16_t v)
{
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((uint16_t)v >> 8);
}

static void example_handle_gen_fan_msg_recieve(esp_ble_mesh_model_t *model,
        esp_ble_mesh_msg_ctx_t *ctx,
        esp_ble_mesh_server_recv_gen_level_set_t *set)
{
	 esp_ble_mesh_gen_level_srv_t *lvl =model->user_data;

	 if (uart_recieve_flag==true)
	 {
		 recieved_control_data=ReceivedByte;
		 if(((ReceivedByte)&(0b10000000))==0X80)
		              	{L1=1;}
		              	if(((ReceivedByte)&(0b10000000))==0)
		              	{L1=0;}
		              	if(((ReceivedByte)&(0b01000000))==0x40)
		              	{
		              		L2=1;
		              	}
		              	if(((ReceivedByte)&(0b01000000))==0)
		              	{
		              		L2=0;
		              	}
		              	if(((ReceivedByte)&(0b00100000))==0x20)
		              	{
		              		L3=1;
		              	}
		              	if(((ReceivedByte)&(0b00100000))==0)
		              	{
		              		L3=0;
		              	}
		              	if(((ReceivedByte)&(0b00010000))==0X10)
		              	{
		              		L4=1;
		              	}
		              	if(((ReceivedByte)&(0b00010000))==0)
		              	{
		              		L4=0;
		              	}
		              	if(((ReceivedByte)&(0b00001000))==0X08)
		              	{
		              		F=1;
		              	}
		              	if(((ReceivedByte)&(0b00001000))==0)
		              	{
		              		F=0;
		              	}
		              	if(((ReceivedByte)&(0b00000001))==0X01)
		              	{
		              		FS1=1;

		              	}

		              	if(((ReceivedByte)&(0b00000001))==0X00)
		              	{
		              		FS1=0;

		              	}
		              	if(((ReceivedByte)&(0b00000010))==0X02)
		              	{
		              		FS2=1;

		              	}
		              	if(((ReceivedByte)&(0b00000010))==0)
		              	{
		              		FS2=0;
		              	}
		              	if(((ReceivedByte)&(0b00000011))==0X03)
		              	{
		              		FS1=1;
		              		FS2=1;
		              	}
		              	if(((ReceivedByte)&(0b00000011))==0)
		              	{
		              		FS1=0;
		              		FS2=0;
		              	}
		              	if(((ReceivedByte)&(0b00000100))==0X04)
		              	{
		              		FS3=1;
		              	}
		              	if(((ReceivedByte)&(0b00000100))==0)
		              	{
		              		FS3=0;
		              	}
		              	if(((ReceivedByte)&(0b00000101))==0X05)
		              	{
		              		FS1=1;
		              		FS3=1;
		              	}
		              	if(((ReceivedByte)&(0b00000101))==0)
		              	{
		              		FS1=0;
		              		FS3=0;
		              	}
		              	if ((FS3==0)&&(FS2==0)&&(FS1==0))
		              			              	                           		{
		              			              		lvl->state.level=-32768;
		              			              	                           		}

		              	 if ((FS3==0)&&(FS2==0)&&(FS1==1))
		              	                           		{
		              		lvl->state.level=-19661;
		              	                           		}
		              	                           		if((FS3==0)&&(FS2==1)&&(FS1==0))
		              	                           		{
		              	                           		lvl->state.level=-6554;
		              	                           		}
		              	                           		if((FS3==0)&&(FS2==1)&&(FS1==1))
		              	                           		{
		              	                           		lvl->state.level=6553;
		              	                           		}
		              	                           		if((FS3==1)&&(FS2==0)&&(FS1==0))
		              	                           		{
		              	                           		lvl->state.level=19660;
		              	                           		}
		              	                           		if((FS3==1)&&(FS2==0)&&(FS1==1))
		              	                           		{
		              	                           		lvl->state.level=32762;
		              	                           		}

		              	                           	control_data=(L1<<7)|(L2<<6)|(L3<<5)|(L4<<4)|(F<<3)|(FS3<<2)|(FS2<<1)|(FS1<<0);

		              	                          uint8_t status[2];
put_le16(status, lvl->state.level);
esp_ble_mesh_server_model_send_msg(model, ctx,
    ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(status), status);
		              	                                  ESP_LOGI(TAG, "get_level_get %d",lvl->state.level );
		              	                                uart_recieve_flag=false;


	 }
	 else
	 {
		 uint8_t status[2];
put_le16(status, lvl->state.level);
esp_ble_mesh_server_model_send_msg(model, ctx,
    ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(status), status);
				              	                                  ESP_LOGI(TAG, "get_level %d",lvl->state.level );
				              	                                control_data=(L1<<7)|(L2<<6)|(L3<<5)|(L4<<4)|(F<<3)|(FS3<<2)|(FS2<<1)|(FS1<<0);
	 }
}

static void example_change_control_state(esp_ble_mesh_model_t *model,
                                     esp_ble_mesh_msg_ctx_t *ctx, int16_t level)
{
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    ESP_LOGI(TAG, "primary address is %x",primary_addr);

    struct _led_state *control=NULL;
    uint8_t i;
    int16_t c=level;
        uint16_t b = 32768;
        uint16_t a=b+c;
        ESP_LOGI(TAG, "level %d", a);

    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst)) {
    	 ESP_LOGI(TAG, "fan address is unicast");
        for (i = 0; i < elem_count; i++) {
            if (ctx->recv_dst == (primary_addr + i)) {
               // led = &led_state[i];
                if(a>0&&a<=13107)
                {
                	FS1=1;
                	FS2=0;
                	FS3=0;
                }
                else if(a>13107&&a<=26214)
                {
                	FS1=0;
                	FS2=1;
                	FS3=0;
                }
                else if(a>26214&&a<=39321)
                              {
                               	FS1=1;
                               	FS2=1;
                               	FS3=0;
                               }
                else if(a>39321&&a<=52428)
                               {
                               	FS1=0;
                               	FS2=0;
                               	FS3=1;
                               }
                else if(a>52428&&a<=65535)
                               {
                               	FS1=1;
                               	FS2=0;
                               	FS3=1;
                               }
                else if(a==0)
                                            {
                                            	FS1=0;
                                            	FS2=0;
                                            	FS3=0;
                                            }
                control_data=(L1<<7)|(L2<<6)|(L3<<5)|(L4<<4)|(F<<3)|(FS3<<2)|(FS2<<1)|(FS1<<0);

            }
        }
    } else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
    	ESP_LOGI(TAG, "fan address is group");
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst)) {
            //led = &led_state[model->element->element_addr - primary_addr];
            //board_led_operation(led->pin, onoff);
            control=&control_state[model->element->element_addr -primary_addr];
            if(a>0&&a<=13107)
                            {
                            	FS1=1;
                            	FS2=0;
                            	FS3=0;
                            }
                            else if(a>13107&&a<=26214)
                            {
                            	FS1=0;
                            	FS2=1;
                            	FS3=0;
                            }
                            else if(a>26214&&a<=39321)
                                          {
                                           	FS1=1;
                                           	FS2=1;
                                           	FS3=0;
                                           }
                            else if(a>39321&&a<=52428)
                                           {
                                           	FS1=0;
                                           	FS2=0;
                                           	FS3=1;
                                           }
                            else if((a>52428)&&(a<=65535))
                                           {
                                           	FS1=1;
                                           	FS2=0;
                                           	FS3=1;
                                           }
                            else if(a==0)
                            {
                            	FS1=0;
                            	FS2=0;
                            	FS3=0;
                            }

                           control_data=(L1<<7)|(L2<<6)|(L3<<5)|(L4<<4)|(F<<3)|(FS3<<2)|(FS2<<1)|(FS1<<0);
        }
    } else if (ctx->recv_dst == 0xFFFF) {
        //led = &led_state[model->element->element_addr - primary_addr];
        //board_led_operation(led->pin, onoff);
        control=&control_state[model->element->element_addr -primary_addr];

        L1=control_state[0].previous;
                       L2=control_state[1].previous;
                       L3=control_state[2].previous;
                       L4=control_state[3].previous;
                       F=control_state[4].previous;
                       S=control_state[5].previous;
    }
}

static void example_handle_gen_fan_msg(esp_ble_mesh_model_t *model,
                                         esp_ble_mesh_msg_ctx_t *ctx,
                                         esp_ble_mesh_server_recv_gen_level_set_t *set)
{
   // esp_ble_mesh_gen_onoff_srv_t *srv = model->user_data;
    esp_ble_mesh_gen_level_srv_t *lvl =model->user_data;


uint8_t status[2];

    switch (ctx->recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET :
     
put_le16(status, lvl->state.level);
esp_ble_mesh_server_model_send_msg(model, ctx,
    ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(status), status);
        ESP_LOGI(TAG, "get_level %d",lvl->state.level );
        break;
    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK:
        if (set->op_en == false) {
            lvl->state.level = set->level;
        } else {
            /* TODO: Delay and state transition */
            lvl->state.level = set->level;
        }
        if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET) {
            uint8_t status[2];
put_le16(status, lvl->state.level);
esp_ble_mesh_server_model_send_msg(model, ctx,
    ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(status), status);
        }       
status[0] = (uint8_t)(lvl->state.level & 0xFF);
status[1] = (uint8_t)((lvl->state.level >> 8) & 0xFF);

esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS,
            sizeof(status), status, ROLE_NODE);
        example_change_control_state(model, ctx, lvl->state.level);
        break;
    default:
        break;
    }


}



static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                               esp_ble_mesh_generic_server_cb_param_t *param)
{
    esp_ble_mesh_gen_onoff_srv_t *srv;
    esp_ble_mesh_gen_level_srv_t *lvl;
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x", param->value.state_change.onoff_set.onoff);
            example_change_led_state(param->model, &param->ctx, param->value.state_change.onoff_set.onoff);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            srv = (esp_ble_mesh_gen_onoff_srv_t *)param->model->user_data;
            ESP_LOGI(TAG, "onoff 0x%02x", srv->state.onoff);
            example_handle_gen_onoff_msg(param->model, &param->ctx, NULL);
        }
          if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET) {
                    lvl = param->model->user_data;
                    ESP_LOGI(TAG, "level %d", lvl->state.level);
                    example_handle_gen_fan_msg_recieve(param->model, &param->ctx, NULL);
                }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x, tid 0x%02x", param->value.set.onoff.onoff, param->value.set.onoff.tid);
            if (param->value.set.onoff.op_en) {
                ESP_LOGI(TAG, "trans_time 0x%02x, delay 0x%02x",
                    param->value.set.onoff.trans_time, param->value.set.onoff.delay);
            }
            example_handle_gen_onoff_msg(param->model, &param->ctx, &param->value.set.onoff);
        }
           if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET ||
                    param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK) {
                    ESP_LOGI(TAG, "level %d, tid 0x%02x", param->value.set.level.level, param->value.set.level.tid);
                    if (param->value.set.level.op_en) {
                        ESP_LOGI(TAG, "trans_time 0x%02x, delay 0x%02x",
                            param->value.set.level.trans_time, param->value.set.level.delay);
                    }
                    example_handle_gen_fan_msg(param->model, &param->ctx, &param->value.set.level);
                }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    board_led_operation(LED_G, LED_ON);

    return err;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    board_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    //  xTaskCreate(System_operation_task, "operation_task", 1024*2, NULL, 15, NULL);
    // init();
    // xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
   // uart_rx_start();
}


// void init() {
//     const uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//     };
//     uart_param_config(UART_NUM_0, &uart_config);
//         uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//         // We won't use a buffer for sending data.
//         uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
//     }


// static void rx_task(void *arg)

// {
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);



//     while (1) {
//       const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);

//         if (rxBytes > 0) {
//         	uart_recieve_flag=true;
//             data[rxBytes] = 0;
//             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//             ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
//             ESP_LOGI("data recieved","read %d value",data[0]);


//             ReceivedByte=data[0];
//             control_data=ReceivedByte;


//             control_state[0].previous=L1;
//                            control_state[1].previous=L2;
//                            control_state[2].previous=L3;
//                            control_state[3].previous=L4;
//                            control_state[4].previous=F;





//         }
//     }
//     free(data);
//     vTaskDelay(10 / portTICK_PERIOD_MS);
//}


// void System_operation_task(void *arg)

// {
//     /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
//        muxed to GPIO on reset already, but some default to other
//        functions and need to be switched to GPIO. Consult the
//        Technical Reference for a list of pads and their default
//        functions.)
//     */
//     // gpio_pad_select_gpio(RELAY1);
//     // gpio_pad_select_gpio(RELAY2);
//     // gpio_pad_select_gpio(RELAY3);
//     // gpio_pad_select_gpio(RELAY4);
//     // gpio_pad_select_gpio(RELAY5);
//     // gpio_pad_select_gpio(RELAY6);
//     // gpio_pad_select_gpio(RELAY7);
//     // gpio_pad_select_gpio(RELAY8);
//     // gpio_pad_select_gpio(RELAY9);
//     // gpio_pad_select_gpio(DATA_PIN);
//     // gpio_pad_select_gpio(CLOCK_PIN);
//     // gpio_pad_select_gpio(LATCH_PIN);


//     /* Set the GPIO as a push/pull output */
//     gpio_set_direction(RELAY1, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RELAY2, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RELAY3, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RELAY4, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RELAY5, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RELAY6, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RELAY7, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RELAY8, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RELAY9, GPIO_MODE_OUTPUT);
//     gpio_set_direction(DATA_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_direction(CLOCK_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_direction(LATCH_PIN, GPIO_MODE_OUTPUT);
//     while(1) {
//     	//vTaskSuspend(NULL);
//     	vTaskDelay(10 / portTICK_PERIOD_MS);
//         /* Blink off (output low) */
//     	 bool L1_State = L1;
//     	  if (L1_State != last_L1_State){
//     		  last_L1_State = L1_State;
//         gpio_set_level(RELAY1, L1_State);

//     	  }

//         bool L2_State = L2;
//             	  if (L2_State != last_L2_State){
//             		  last_L2_State = L2_State;
//                 gpio_set_level(RELAY2, L2_State);
//             	  }

//                 bool L3_State = L3;
//                 if (L3_State != last_L3_State){
//                 last_L3_State = L3_State;
//                 gpio_set_level(RELAY3, L3_State);
//                 }
//            bool L4_State = L4;
//            if (L4_State != last_L4_State){
//            last_L4_State = L4_State;
//            gpio_set_level(RELAY4, L4_State);
//           }

//            bool F_State = F;
//            if (F_State != last_F_State){
//            last_F_State = F_State;
//           // gpio_set_level(RELAY1, F_State);
//            }
//            uint8_t shift_value=(L1<<7)|(L2<<6)|(L3<<5)|(L4<<4)|(F<<3)|(S<<2)|(0<<1)|(0<<0);
//            gpio_set_level(LATCH_PIN, 0);
//           //         shift_Out(DATA_PIN,CLOCK_PIN,LSBFIRST,shift_value);
//                    gpio_set_level(LATCH_PIN, 1);

//            bool F1_State = FS1;
//            bool F2_State = FS2;
//            bool F3_State = FS3;
//            if (F1_State != last_F1_State||F2_State != last_F2_State||F3_State != last_F3_State){
//              last_F1_State = F1_State;
//              last_F2_State = F2_State;
//              last_F3_State = F3_State;
//              if ((F3_State==0)&&(FS2==0)&&(F1_State==1)&&(F==1))
//              		{
//              			printf("FAN SPEED 1 \n");
//              			gpio_set_level(RELAY5, 1);
//              			gpio_set_level(RELAY6, 0);
//              			gpio_set_level(RELAY7, 0);
//              			gpio_set_level(RELAY8, 0);
//              			gpio_set_level(5, 1);
//              			gpio_set_level(32, 0);
//              			gpio_set_level(33, 0);


//              		}
//              else if((F3_State==0)&&(FS2==1)&&(F1_State==0)&&(F==1))
//              {
//             	 printf("FAN SPEED 2 \n");
//             	 gpio_set_level(RELAY5, 1);
//             	              			gpio_set_level(RELAY6, 1);
//             	              			gpio_set_level(RELAY7, 0);
//             	              			gpio_set_level(RELAY8, 0);
//             	              			gpio_set_level(32, 1);
//             	              			gpio_set_level(33, 0);
//             	              			gpio_set_level(5, 0);

//              }
//              else if((F3_State==0)&&(FS2==1)&&(F1_State==1)&&(F==1))
//              {
//             	 printf("FAN SPEED 3 \n");
//             	 gpio_set_level(RELAY5, 0);
//             	              			gpio_set_level(RELAY6, 1);
//             	              			gpio_set_level(RELAY7, 1);
//             	              			gpio_set_level(RELAY8, 0);
//             	              			gpio_set_level(33, 1);
//             	              			gpio_set_level(32, 0);
//             	              			gpio_set_level(5, 0);
//              }

//              else if((F3_State==1)&&(FS2==0)&&(F1_State==0)&&(F==1))
//              {
//             	 printf("FAN SPEED 4 \n");
//             	 gpio_set_level(RELAY5, 1);
//             	              			gpio_set_level(RELAY6, 1);
//             	              			gpio_set_level(RELAY7, 1);
//             	              			gpio_set_level(RELAY8, 0);
//             	              			gpio_set_level(5, 1);
//             	              			gpio_set_level(32, 1);
//             	              			gpio_set_level(33, 0);
//              }
//              else if((F3_State==1)&&(FS2==0)&&(F1_State==1)&&(F==1))
//              {
//             	 printf("FAN SPEED 5 \n");
//             	 gpio_set_level(RELAY5, 0);
//             	              			gpio_set_level(RELAY6, 0);
//             	              			gpio_set_level(RELAY7, 0);
//             	              			gpio_set_level(RELAY8, 1);
//             	              			gpio_set_level(33, 1);
//             	              			gpio_set_level(5, 1);
//             	              			gpio_set_level(32, 0);
//              }
//              else
//              {
//             	 gpio_set_level(RELAY5, 0);
//             	             	              			gpio_set_level(RELAY6, 0);
//             	             	              			gpio_set_level(RELAY7, 0);
//             	             	              			gpio_set_level(RELAY8, 0);
//             	             	              			gpio_set_level(33, 0);
//             	             	              			gpio_set_level(5, 0);

//             	             	              			gpio_set_level(32, 0);
//             	              }
//              }
//            bool S_State = S;
//                               if (S_State != last_S_State){
//                               last_S_State = S_State;
//                               gpio_set_level(RELAY9, S_State);

//               }

//             }

//       //  vTaskDelay(1000 / portTICK_PERIOD_MS);
//         /* Blink on (output high) */
//        // gpio_set_level(BLINK_GPIO, 1);
//       //  vTaskDelay(1000 / portTICK_PERIOD_MS);
//       vTaskDelay(10 / portTICK_PERIOD_MS);
//     }