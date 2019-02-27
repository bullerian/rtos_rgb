/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *******************************************************************************/

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/pwm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#include "driver/pwm.h"

#include "MQTTClient.h"

#include  "jsmn.h"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

#define MQTT_CLIENT_THREAD_NAME         "mqtt_client_thread"
#define MQTT_CLIENT_THREAD_STACK_WORDS  4096
#define MQTT_CLIENT_THREAD_PRIO         8
#define PIN_STATE 2
#define BUFF_LEN 100

#define PWM_PIN_RED      15
#define PWM_PIN_BLUE     12
#define PWM_PIN_GREEN    14
#define PWM_PERIOD    (500)

#define TRUE_S  "True"

typedef struct {
    uint8_t r, g, b;
    bool is_on;
    bool is_sensor_on;
} led_params_s;

led_params_s led_param;

const uint32_t pin_num[3] = {
    PWM_PIN_GREEN,
    PWM_PIN_BLUE,
    PWM_PIN_RED
};

// dutys table, (duty/PERIOD)*depth
uint32_t duties[] = {
    0, 0, 0
};

// phase table, (phase/180)*depth
int16_t phase[] = {
    0, 0, 0
};

char json_buff[BUFF_LEN] = {0};

const uint8_t gamma_table[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

static const char *TAG = "example";
static QueueHandle_t RawJson;
static char* data;

static char* test_str = "{\"r\": 128, \"g\": 200, \"b\": 50, \"is_on\": True, \"sensor\": False}";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;

    default:
        break;
    }

    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
        if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
                        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
                return 0;
        }
        return -1;
}

static void messageArrived(MessageData *data)
{
    // ESP_LOGI(TAG, "Message arrived[len:%u]: %.*s", data->message->payloadlen, data->message->payloadlen, (char *)data->message->payload);
    // ESP_LOGI(TAG, "Message arrived[len:%u]: %s", data->message->payloadlen, (char *)data->message->payload);

    strncpy(json_buff, (char *)data->message->payload, data->message->payloadlen);
    uint32_t len = data->message->payloadlen;

    ESP_LOGI(TAG, "Message arrived: %s, len: %d", json_buff, data->message->payloadlen);

    data = &json_buff[0];
    if(len < (BUFF_LEN - 1))
    {
        json_buff[len] = '\0';
    }

    int rc = xQueueSend(RawJson, &data, 0);

    ESP_LOGI(TAG, "%d", (uint32_t) &json_buff[0]);

    if(rc == pdPASS)
    {
        ESP_LOGI(TAG, "SENT!");
    }
}

static bool str_to_bool(char* start, size_t len)
{
    return !(bool) strncmp(TRUE_S, start, len);
}

static uint8_t process_json(char * text)
{
    size_t i;
	int r;
	jsmn_parser p;
	jsmntok_t t[128]; /* We expect no more than 128 tokens */

	jsmn_init(&p);
	r = jsmn_parse(&p, text, strlen(text), t, sizeof(t)/sizeof(t[0]));
	if (r < 0) {
		ESP_LOGE(TAG, "Failed to parse JSON: %d\n", r);
		return 1;
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		ESP_LOGE(TAG, "Object expected\n");
		return 2;
	}

	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++) {
		if (jsoneq(text, &t[i], "r") == 0) {
			/* We may use strndup() to fetch string value */
            led_param.r = (uint8_t) strtol(text + t[i+1].start, t[i+1].end-t[i+1].start, 10);
			ESP_LOGD(TAG, "Red: %d\n", led_param.r);
			i++;
            }
		else if (jsoneq(text, &t[i], "g") == 0) {
			/* We may use strndup() to fetch string value */
            led_param.g = (uint8_t) strtol(text + t[i+1].start, t[i+1].end-t[i+1].start, 10);
			ESP_LOGD(TAG, "Green: %d", led_param.g);
			i++;
            }
		else if (jsoneq(text, &t[i], "b") == 0) {
			/* We may use strndup() to fetch string value */
            led_param.b = (uint8_t) strtol(text + t[i+1].start, t[i+1].end-t[i+1].start, 10);
			ESP_LOGD(TAG, "Blue: %d\n", led_param.b);
			i++;
            }
		else if (jsoneq(text, &t[i], "is_on") == 0) {
			/* We may use strndup() to fetch string value */
            led_param.is_on = str_to_bool(text + t[i+1].start, t[i+1].end-t[i+1].start);

			ESP_LOGD(TAG, "is_on: %d\n", led_param.is_on);
			i++;
            }
        else if (jsoneq(text, &t[i], "sensor") == 0) {
			/* We may use strndup() to fetch string value */
            led_param.is_sensor_on = str_to_bool(text + t[i+1].start, t[i+1].end-t[i+1].start);
			ESP_LOGD(TAG, "sensor: %d\n", led_param.is_sensor_on);
			i++;
            } 
        else {
			ESP_LOGE(TAG, "Unexpected key: %.*s\n", t[i].end-t[i].start,
					text + t[i].start);
            return 3;
		    }
    }

    // ESP_LOGI(TAG, "r=%d\ng=%d\nb=%d\nis_on=%d\nis_sensor=%d", led_param.r, led_param.g, led_param.b, led_param.is_on, led_param.is_sensor_on);
    
    return 0;
}

void mqtt_client_thread(void *pvParameters)
{
    char *payload = NULL;
    MQTTClient client;
    Network network;
    int rc = 0;
    char clientID[32] = {0};
    uint32_t count = 0;

    ESP_LOGI(TAG, "ssid:%s passwd:%s sub:%s qos:%u pub:%s qos:%u pubinterval:%u payloadsize:%u",
             CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD, CONFIG_MQTT_SUB_TOPIC,
             CONFIG_DEFAULT_MQTT_SUB_QOS, CONFIG_MQTT_PUB_TOPIC, CONFIG_DEFAULT_MQTT_PUB_QOS,
             CONFIG_MQTT_PUBLISH_INTERVAL, CONFIG_MQTT_PAYLOAD_BUFFER);

    ESP_LOGI(TAG, "ver:%u clientID:%s keepalive:%d username:%s passwd:%s session:%d level:%u",
             CONFIG_DEFAULT_MQTT_VERSION, CONFIG_MQTT_CLIENT_ID,
             CONFIG_MQTT_KEEP_ALIVE, CONFIG_MQTT_USERNAME, CONFIG_MQTT_PASSWORD,
             CONFIG_DEFAULT_MQTT_SESSION, CONFIG_DEFAULT_MQTT_SECURITY);

    ESP_LOGI(TAG, "broker:%s port:%u", CONFIG_MQTT_BROKER, CONFIG_MQTT_PORT);

    ESP_LOGI(TAG, "sendbuf:%u recvbuf:%u sendcycle:%u recvcycle:%u",
             CONFIG_MQTT_SEND_BUFFER, CONFIG_MQTT_RECV_BUFFER,
             CONFIG_MQTT_SEND_CYCLE, CONFIG_MQTT_RECV_CYCLE);

    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

    NetworkInit(&network);

    if (MQTTClientInit(&client, &network, 0, NULL, 0, NULL, 0) == false) {
        ESP_LOGE(TAG, "mqtt init err");
        vTaskDelete(NULL);
    }

    payload = malloc(CONFIG_MQTT_PAYLOAD_BUFFER);

    if (!payload) {
        ESP_LOGE(TAG, "mqtt malloc err");
    } else {
        memset(payload, 0x0, CONFIG_MQTT_PAYLOAD_BUFFER);
    }

    for (;;) {
        ESP_LOGI(TAG, "wait wifi connect...");
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

        if ((rc = NetworkConnect(&network, CONFIG_MQTT_BROKER, CONFIG_MQTT_PORT)) != 0) {
            ESP_LOGE(TAG, "Return code from network connect is %d", rc);
            continue;
        }

        connectData.MQTTVersion = CONFIG_DEFAULT_MQTT_VERSION;

        sprintf(clientID, "%s_%u", CONFIG_MQTT_CLIENT_ID, esp_random());

        connectData.clientID.cstring = clientID;
        connectData.keepAliveInterval = CONFIG_MQTT_KEEP_ALIVE;

        connectData.username.cstring = CONFIG_MQTT_USERNAME;
        connectData.password.cstring = CONFIG_MQTT_PASSWORD;

        connectData.cleansession = CONFIG_DEFAULT_MQTT_SESSION;

        ESP_LOGI(TAG, "MQTT Connecting");

        if ((rc = MQTTConnect(&client, &connectData)) != 0) {
            ESP_LOGE(TAG, "Return code from MQTT connect is %d", rc);
            network.disconnect(&network);
            continue;
        }

        ESP_LOGI(TAG, "MQTT Connected");

        if ((rc = MQTTSubscribe(&client, CONFIG_MQTT_SUB_TOPIC, CONFIG_DEFAULT_MQTT_SUB_QOS, messageArrived)) != 0) {
            ESP_LOGE(TAG, "Return code from MQTT subscribe is %d", rc);
            network.disconnect(&network);
            continue;
        }

        if ((rc = MQTTStartTask(&client)) != pdPASS) {
            ESP_LOGE(TAG, "Return code from start tasks is %d", rc);
        }

        while(1){
            taskYIELD();
            // vTaskDelay(pdMS_TO_TICKS(6000));
        }
    }

    ESP_LOGW(TAG, "mqtt_client_thread going to be deleted");
    vTaskDelete(NULL);
    return;
}

void vBlink(void* vParams)
{
    gpio_config_t led_pin;
    led_pin.intr_type = GPIO_INTR_DISABLE;
    led_pin.mode = GPIO_MODE_OUTPUT;
    led_pin.pin_bit_mask = 1ULL<<PIN_STATE;
    led_pin.pull_down_en = false;
    led_pin.pull_up_en = false;
    gpio_config(&led_pin);

    while (1)
    {
        gpio_set_level(PIN_STATE, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(PIN_STATE, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void vLed(void *arg){
    uint8_t i = 0;
    uint8_t ctr = 0;
    pwm_init(PWM_PERIOD, duties, 3, pin_num);
    pwm_set_phases(phase);

    pwm_start();

    while(1){
        vTaskDelay(pdMS_TO_TICKS(10));
        if(ctr != 255){
            duties[i] = gamma_table[ctr++];
            pwm_set_duties(duties);
        } else {
            ESP_LOGI(TAG, "Clearing duties. G=%d B=%d R=%d", duties[0], duties[1], duties[2]);
            duties[i] = 0;
            ctr = 0;
            pwm_set_duties(duties);
            i++;
            if(i >= 3){
                i = 0;
            }
        }
        pwm_start();
    }
}

void vPrint(void* vParams)
{
    int rc = 0;
    char* c;

    ESP_LOGI(TAG, "Waiting for messages");
    while (1)
    {
        // ESP_LOGI(TAG, "Poling queue");
        rc = xQueueReceive(RawJson, &c, portMAX_DELAY);
        if (rc != pdPASS)
        {
            ESP_LOGE(TAG, "Queue receive failed");
        } else {
            // if (*data != json_buff[0]){
            //     ESP_LOGI(TAG, "wrong val");
            // }
            // printf("%02x\n", (uint32_t) &json_buff[0]);
            ESP_LOGI(TAG, "Queue receive OK: %s", c);

            process_json(c);
                      //memset(data, 0, BUFF_LEN);
        }
    }


}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    RawJson = xQueueCreate(3, sizeof(char*));
    if( RawJson == NULL )
    {
        ESP_LOGE(TAG, "QUEUE creation failed!!!!");
        while(1);
    }


    // char* c;
    // char* data;
    // int rc = 0;

    // strcpy(json_buff, "{\"test\": 44}");

    // c = &json_buff[0];

    // while(1){
    //     rc = xQueueSend(RawJson, &c, 0);
    //     if (rc == pdPASS){
    //         ESP_LOGI(TAG, "Sent");
    //     }

    //     ESP_LOGI(TAG, "Poling queue");
    //     rc = xQueueReceive(RawJson, &data, 0);
    //     if (rc != pdPASS)
    //     {
    //         ESP_LOGE(TAG, "Queue receive failed");
    //     } else {
    //         ESP_LOGI(TAG, "Queue receive OK, %s", data);
    //         //memset(data, 0, BUFF_LEN);
    //     }
    // }

    initialise_wifi();

    // ret = xTaskCreate(&vBlink, "Blink1", 2048, NULL, 2, NULL);
    ret = xTaskCreate(&vPrint, "Print1", 2048 * 2, NULL, 9, NULL);
    ret = xTaskCreate(&vLed, "PWM", 2048, NULL, 8, NULL);

    ret = xTaskCreate(&mqtt_client_thread,
                      MQTT_CLIENT_THREAD_NAME,
                      MQTT_CLIENT_THREAD_STACK_WORDS,
                      NULL,
                      MQTT_CLIENT_THREAD_PRIO,
                      NULL);

    if (ret != pdPASS)  {
        ESP_LOGE(TAG, "mqtt create client thread %s failed", MQTT_CLIENT_THREAD_NAME);
    }
}
