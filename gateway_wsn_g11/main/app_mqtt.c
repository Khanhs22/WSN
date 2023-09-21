#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "driver/gpio.h"

static const char *TAG = "MQTT_app";

extern bool cb_mqtt_1;
extern bool cb_mqtt_2;
extern bool cb_mqtt_3;
extern bool cb_mqtt_4;
extern bool cb_mqtt_5;
extern bool cb_mqtt_6;
extern bool cb_mqtt_7;

extern char data1[4];
extern char data2[4];
extern char data3[4];
extern char data4[4];
extern char data5[4];
extern char data6[4];
extern char data7[4];

bool flag_cb_mqtt = false;
int len_cb_mqtt = 0;
char *data_cb_mqtt = NULL;

esp_mqtt_client_handle_t client;

//callback functions when receive data from nodes
static void cb_send_data1(void* arg)
{
    for(;;) {
        if(cb_mqtt_1)
        {
            char data[30];
            sprintf(data, "{\"temperature1\":%s}", data1);
            cb_mqtt_1 = 0;
            esp_mqtt_client_publish(client, "v1/devices/me/telemetry", data, 0, 1, 0);
        }
        vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
}

static void cb_send_data2(void* arg)
{
    for(;;) {
        if(cb_mqtt_2)
        {
            char data[30];
            sprintf(data, "{\"temperature2\":%.*s}", 4, data2);
            cb_mqtt_2 = 0;
            esp_mqtt_client_publish(client, "v1/devices/me/telemetry", data, 0, 1, 0);
        }
        vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
}

static void cb_send_data3(void* arg)
{
    for(;;) {
        if(cb_mqtt_3)
        {
            char data[30];
            sprintf(data, "{\"temperature3\":%s}", data3);
            cb_mqtt_3 = 0;
            esp_mqtt_client_publish(client, "v1/devices/me/telemetry", data, 0, 1, 0);
        }
        vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
}

static void cb_send_data4(void* arg)
{
    for(;;) {
        if(cb_mqtt_4)
        {
            char data[30];
            sprintf(data, "{\"temperature4\":%s}", data4);
            cb_mqtt_4 = 0;
            esp_mqtt_client_publish(client, "v1/devices/me/telemetry", data, 0, 1, 0);
        }
        vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
}

static void cb_send_data5(void* arg)
{
    for(;;) {
        if(cb_mqtt_5)
        {
            char data[30];
            sprintf(data, "{\"temperature5\":%s}", data5);
            cb_mqtt_5 = 0;
            esp_mqtt_client_publish(client, "v1/devices/me/telemetry", data, 0, 1, 0);
        }
        vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
}

static void cb_send_data6(void* arg)
{
    for(;;) {
        if(cb_mqtt_6)
        {
            char data[30];
            sprintf(data, "{\"temperature6\":%s}", data6);
            cb_mqtt_6 = 0;
            esp_mqtt_client_publish(client, "v1/devices/me/telemetry", data, 0, 1, 0);
        }
        vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
}

static void cb_send_data7(void* arg)
{
    for(;;) {
        if(cb_mqtt_7)
        {
            char data[30];
            sprintf(data, "{\"temperature7\":%s}", data7);
            cb_mqtt_7 = 0;
            esp_mqtt_client_publish(client, "v1/devices/me/telemetry", data, 0, 1, 0);      
        }
        vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

//callback function when receive set_up data from clould sever
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // msg_id = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", "{\"temperature\":42}", 0, 1, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "v1/devices/me/attributes", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        // msg_id = esp_mqtt_client_publish(client, "/IOT/DHT11", "node subcribed", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        // printf("DATA=%.*s\r\n", event->data_len, event->data);
        len_cb_mqtt = event->data_len;
        data_cb_mqtt = event->data;
        if(strstr(data_cb_mqtt, "\"set_up\""))
        {
            //strncpy(data_cb_mqtt, get_control + 10, 6); 
            //printf("%s\n", data_cb_mqtt);
            flag_cb_mqtt = true;
        } 
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(void) //start mqtt
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://thingsboard.cloud",
        .port= 1883,
        .username="JBhK2icllZnWctGSgtWT",
        // .password=""
    };

    client = esp_mqtt_client_init(&mqtt_cfg);

    xTaskCreate(cb_send_data1, "cb_send_data1", 4096, NULL, 10, NULL);
    xTaskCreate(cb_send_data2, "cb_send_data2", 4096, NULL, 10, NULL);
    xTaskCreate(cb_send_data3, "cb_send_data3", 4096, NULL, 10, NULL);
    xTaskCreate(cb_send_data4, "cb_send_data4", 4096, NULL, 10, NULL);
    xTaskCreate(cb_send_data5, "cb_send_data5", 4096, NULL, 10, NULL);
    xTaskCreate(cb_send_data6, "cb_send_data6", 4096, NULL, 10, NULL);
    xTaskCreate(cb_send_data7, "cb_send_data7", 4096, NULL, 10, NULL);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    vTaskDelay(2000/ portTICK_PERIOD_MS);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}