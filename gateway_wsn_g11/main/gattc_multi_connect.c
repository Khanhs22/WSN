/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_config.h"
#include "app_mqtt.h"
#include "esp_intr_alloc.h"

extern bool flag_cb_mqtt;
extern int len_cb_mqtt;
extern char *data_cb_mqtt;

//callback values from nodes
bool cb_mqtt_1 = 0;
bool cb_mqtt_2 = 0;
bool cb_mqtt_3 = 0;
bool cb_mqtt_4 = 0;
bool cb_mqtt_5 = 0;
bool cb_mqtt_6 = 0;
bool cb_mqtt_7 = 0;

//data from nodes
char data1[4];
char data2[4];
char data3[4];
char data4[4];
char data5[4];
char data6[4];
char data7[4];

#define GATTC_TAG "GATTC_MULTIPLE_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01

/* register three profiles, each profile corresponds to one connection,
   which makes it easy to handle each connection event */
#define PROFILE_NUM 7
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1
#define PROFILE_C_APP_ID 2
#define PROFILE_D_APP_ID 3
#define PROFILE_E_APP_ID 4
#define PROFILE_F_APP_ID 5
#define PROFILE_G_APP_ID 6
#define INVALID_HANDLE   0

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_b_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_c_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_d_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_e_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_f_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_g_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

//information of nodes
static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static bool conn_device_a   = false;
static bool conn_device_b   = false;
static bool conn_device_c   = false;
static bool conn_device_d   = false;
static bool conn_device_e   = false;
static bool conn_device_f   = false;
static bool conn_device_g   = false;


static bool get_service_a   = false;
static bool get_service_b   = false;
static bool get_service_c   = false;
static bool get_service_d   = false;
static bool get_service_e   = false;
static bool get_service_f   = false;
static bool get_service_g   = false;

static bool Isconnecting    = false;
static bool stop_scan_done  = false;

static esp_gattc_char_elem_t  *char_elem_result_a   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result_a  = NULL;
static esp_gattc_char_elem_t  *char_elem_result_b   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result_b  = NULL;
static esp_gattc_char_elem_t  *char_elem_result_c   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result_c  = NULL;
static esp_gattc_char_elem_t  *char_elem_result_d   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result_d  = NULL;
static esp_gattc_char_elem_t  *char_elem_result_e   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result_e  = NULL;
static esp_gattc_char_elem_t  *char_elem_result_f   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result_f  = NULL;
static esp_gattc_char_elem_t  *char_elem_result_g   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result_g  = NULL;

//name of nodes (which gateway will scan and request to connect)
static const char remote_device_name[7][20] = {"ESP_GATTS_DEMO_1", "ESP_GATTS_DEMO_2", "ESP_GATTS_DEMO_3", "ESP_GATTS_DEMO_4", 
                                                        "ESP_GATTS_DEMO_5", "ESP_GATTS_DEMO_6", "ESP_GATTS_DEMO_7"};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_a_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_B_APP_ID] = {
        .gattc_cb = gattc_profile_b_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_C_APP_ID] = {
        .gattc_cb = gattc_profile_c_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_D_APP_ID] = {
        .gattc_cb = gattc_profile_d_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_E_APP_ID] = {
        .gattc_cb = gattc_profile_e_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_F_APP_ID] = {
        .gattc_cb = gattc_profile_f_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_G_APP_ID] = {
        .gattc_cb = gattc_profile_g_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};

static void start_scan(void)
{
    stop_scan_done = false;
    Isconnecting = false;
    uint32_t duration = 30;
    esp_ble_gap_start_scanning(duration);
}

//handle for node 1
static void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{

    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    /* one device connect successfully, all profiles callback function will get the ESP_GATTC_CONNECT_EVT,
     so must compare the mac address to check which device is connected, so it is a good choice to use ESP_GATTC_OPEN_EVT. */
    case ESP_GATTC_CONNECT_EVT:
        break;
    case ESP_GATTC_OPEN_EVT:
        if (p_data->open.status != ESP_GATT_OK){
            //open failed, ignore the first device, connect the second device
            ESP_LOGE(GATTC_TAG, "connect device failed, status %d", p_data->open.status);
            conn_device_a = false;
            //start_scan();
            break;
        }
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->open.remote_bda, 6);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->open.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"Config mtu failed");
        }
        ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            get_service_a = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (get_service_a){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0) {
                char_elem_result_a = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result_a){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else {
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result_a,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result_a[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result_a[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result_a[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result_a);
            }else {
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        start_scan();
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "reg notify failed, error status =%x", p_data->reg_for_notify.status);
            break;
        }
        uint16_t count = 0;
        // uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     ESP_GATT_DB_DESCRIPTOR,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                     &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }
        if (count > 0){
            descr_elem_result_a = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result_a){
                ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
            }else{
                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     p_data->reg_for_notify.handle,
                                                                     notify_descr_uuid,
                                                                     descr_elem_result_a,
                                                                     &count);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                }

                /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                // if (count > 0 && descr_elem_result_a[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result_a[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                //     ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                //                                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                //                                                  descr_elem_result_a[0].handle,
                //                                                  sizeof(notify_en),
                //                                                  (uint8_t *)&notify_en,
                //                                                  ESP_GATT_WRITE_TYPE_RSP,
                //                                                  ESP_GATT_AUTH_REQ_NONE);
                // }

                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }

                /* free descr_elem_result */
                free(descr_elem_result_a);
            }
        }
        else{
            ESP_LOGE(GATTC_TAG, "decsr not found");
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, Receive notify value:");
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        
        char data_get1[4] = {p_data->notify.value[0], p_data->notify.value[1], p_data->notify.value[2], p_data->notify.value[3]};
        printf("Data from node 1: %s\n", data_get1);
        strcpy(data1, data_get1);
        cb_mqtt_1 = 1;
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success");
        // uint8_t write_char_data[35];
        // for (int i = 0; i < sizeof(write_char_data); ++i)
        // {
        //     write_char_data[i] = i % 256;
        // }
        // esp_ble_gattc_write_char( gattc_if,
        //                           gl_profile_tab[PROFILE_A_APP_ID].conn_id,
        //                           gl_profile_tab[PROFILE_A_APP_ID].char_handle,
        //                           sizeof(write_char_data),
        //                           write_char_data,
        //                           ESP_GATT_WRITE_TYPE_RSP,
        //                           ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
        }else{
            ESP_LOGI(GATTC_TAG, "write char success");
        }
        // start_scan();
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:%08x%04x",(bda[0] << 24) + (bda[1] << 16) + (bda[2] << 8) + bda[3],
                 (bda[4] << 8) + bda[5]);
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT:
        //Start scanning again
        start_scan();
        if (memcmp(p_data->disconnect.remote_bda, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, 6) == 0){
            ESP_LOGI(GATTC_TAG, "device a disconnect");
            conn_device_a = false;
            get_service_a = false;
        }
        break;
    default:
        break;
    }
}

//handle for node 2
static void gattc_profile_b_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        break;
    case ESP_GATTC_CONNECT_EVT:
        break;
    case ESP_GATTC_OPEN_EVT:
        if (p_data->open.status != ESP_GATT_OK){
            //open failed, ignore the second device, connect the third device
            ESP_LOGE(GATTC_TAG, "connect device failed, status %d", p_data->open.status);
            conn_device_b = false;
            //start_scan();
            break;
        }
        memcpy(gl_profile_tab[PROFILE_B_APP_ID].remote_bda, p_data->open.remote_bda, 6);
        gl_profile_tab[PROFILE_B_APP_ID].conn_id = p_data->open.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"Config mtu failed");
        }
        ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            get_service_b = true;
            gl_profile_tab[PROFILE_B_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_B_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (get_service_b){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result_b = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result_b){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result_b,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result_b[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_B_APP_ID].char_handle = char_elem_result_b[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_B_APP_ID].remote_bda, char_elem_result_b[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result_b);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        start_scan();
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {

        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "reg notify failed, error status =%x", p_data->reg_for_notify.status);
            break;
        }
        uint16_t count = 0;
        //uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].conn_id,
                                                                     ESP_GATT_DB_DESCRIPTOR,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                                     &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }
        if (count > 0){
            descr_elem_result_b = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result_b){
                ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
            }else{
                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].conn_id,
                                                                     p_data->reg_for_notify.handle,
                                                                     notify_descr_uuid,
                                                                     descr_elem_result_b,
                                                                     &count);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                }

                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }

                /* free descr_elem_result */
                free(descr_elem_result_b);
            }
        }
        else{
            ESP_LOGE(GATTC_TAG, "decsr not found");
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, Receive notify value:");
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);

        char data_get2[4] = {p_data->notify.value[0], p_data->notify.value[1], p_data->notify.value[2], p_data->notify.value[3]};
        printf("Data from node 2: %s\n", data_get2);
        strcpy(data2, data_get2);
        cb_mqtt_2 = 1;
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success");

        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Write char failed, error status = %x", p_data->write.status);
        }else{
            ESP_LOGI(GATTC_TAG, "Write char success");
        }
        // start_scan();
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:%08x%04x",(bda[0] << 24) + (bda[1] << 16) + (bda[2] << 8) + bda[3],
                 (bda[4] << 8) + bda[5]);
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT:
        if (memcmp(p_data->disconnect.remote_bda, gl_profile_tab[PROFILE_B_APP_ID].remote_bda, 6) == 0){
            ESP_LOGI(GATTC_TAG, "device b disconnect");
            conn_device_b = false;
            get_service_b = false;
        }
        break;
    default:
        break;
    }
}

//handle for node 3
static void gattc_profile_c_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        break;
    case ESP_GATTC_CONNECT_EVT:
        break;
    case ESP_GATTC_OPEN_EVT:
        if (p_data->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "connect device failed, status %d", p_data->open.status);
            conn_device_c = false;
            //start_scan();
            break;
        }
        memcpy(gl_profile_tab[PROFILE_C_APP_ID].remote_bda, p_data->open.remote_bda, 6);
        gl_profile_tab[PROFILE_C_APP_ID].conn_id = p_data->open.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"Config mtu failed");
        }
        ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            get_service_c = true;
            gl_profile_tab[PROFILE_C_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_C_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (get_service_c){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result_c = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result_c){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_C_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result_c,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result_c[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_C_APP_ID].char_handle = char_elem_result_c[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_C_APP_ID].remote_bda, char_elem_result_c[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result_c);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        start_scan();
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "reg notify failed, error status =%x", p_data->reg_for_notify.status);
            break;
        }
        uint16_t count = 0;
        // uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].conn_id,
                                                                     ESP_GATT_DB_DESCRIPTOR,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].service_end_handle,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].char_handle,
                                                                     &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }
        if (count > 0){
            descr_elem_result_c = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result_c){
                ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
            }else{
                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].conn_id,
                                                                     p_data->reg_for_notify.handle,
                                                                     notify_descr_uuid,
                                                                     descr_elem_result_c,
                                                                     &count);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                }

                /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                // if (count > 0 && descr_elem_result_c[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result_c[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                //     ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                //                                                  gl_profile_tab[PROFILE_C_APP_ID].conn_id,
                //                                                  descr_elem_result_c[0].handle,
                //                                                  sizeof(notify_en),
                //                                                  (uint8_t *)&notify_en,
                //                                                  ESP_GATT_WRITE_TYPE_RSP,
                //                                                  ESP_GATT_AUTH_REQ_NONE);
                // }

                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }

                /* free descr_elem_result */
                free(descr_elem_result_c);
            }
        }
        else{
            ESP_LOGE(GATTC_TAG, "decsr not found");
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, Receive notify value:");
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);

        char data_get3[4] = {p_data->notify.value[0], p_data->notify.value[1], p_data->notify.value[2], p_data->notify.value[3]};
        printf("Data from node 3: %s\n", data_get3);
        strcpy(data3, data_get3);
        cb_mqtt_3 = 1;
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success");
        // uint8_t write_char_data[35];
        // for (int i = 0; i < sizeof(write_char_data); ++i)
        // {
        //     write_char_data[i] = i % 256;
        // }
        // esp_ble_gattc_write_char( gattc_if,
        //                           gl_profile_tab[PROFILE_C_APP_ID].conn_id,
        //                           gl_profile_tab[PROFILE_C_APP_ID].char_handle,
        //                           sizeof(write_char_data),
        //                           write_char_data,
        //                           ESP_GATT_WRITE_TYPE_RSP,
        //                           ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Write char success");
        // start_scan();
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:%08x%04x",(bda[0] << 24) + (bda[1] << 16) + (bda[2] << 8) + bda[3],
                 (bda[4] << 8) + bda[5]);
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT:
        if (memcmp(p_data->disconnect.remote_bda, gl_profile_tab[PROFILE_C_APP_ID].remote_bda, 6) == 0){
            ESP_LOGI(GATTC_TAG, "device c disconnect");
            conn_device_c = false;
            get_service_c = false;
        }
        break;
    default:
        break;
    }
}

//handle for node 4
static void gattc_profile_d_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        break;
    case ESP_GATTC_CONNECT_EVT:
        break;
    case ESP_GATTC_OPEN_EVT:
        if (p_data->open.status != ESP_GATT_OK){
            //open failed, ignore the second device, connect the third device
            ESP_LOGE(GATTC_TAG, "connect device failed, status %d", p_data->open.status);
            conn_device_d = false;
            //start_scan();
            break;
        }
        memcpy(gl_profile_tab[PROFILE_D_APP_ID].remote_bda, p_data->open.remote_bda, 6);
        gl_profile_tab[PROFILE_D_APP_ID].conn_id = p_data->open.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"Config mtu failed");
        }
        ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            get_service_d = true;
            gl_profile_tab[PROFILE_D_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_D_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (get_service_d){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_D_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_D_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result_d = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result_d){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_D_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_D_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result_d,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result_d[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_D_APP_ID].char_handle = char_elem_result_d[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_D_APP_ID].remote_bda, char_elem_result_d[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result_d);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        start_scan();
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {

        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "reg notify failed, error status =%x", p_data->reg_for_notify.status);
            break;
        }
        uint16_t count = 0;
        //uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     gl_profile_tab[PROFILE_D_APP_ID].conn_id,
                                                                     ESP_GATT_DB_DESCRIPTOR,
                                                                     gl_profile_tab[PROFILE_D_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_D_APP_ID].service_end_handle,
                                                                     gl_profile_tab[PROFILE_D_APP_ID].char_handle,
                                                                     &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }
        if (count > 0){
            descr_elem_result_d = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result_d){
                ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
            }else{
                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                     gl_profile_tab[PROFILE_D_APP_ID].conn_id,
                                                                     p_data->reg_for_notify.handle,
                                                                     notify_descr_uuid,
                                                                     descr_elem_result_d,
                                                                     &count);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                }

                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }

                /* free descr_elem_result */
                free(descr_elem_result_d);
            }
        }
        else{
            ESP_LOGE(GATTC_TAG, "decsr not found");
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, Receive notify value:");
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);

        char data_get4[4] = {p_data->notify.value[0], p_data->notify.value[1], p_data->notify.value[2], p_data->notify.value[3]};
        printf("Data from node 4: %s\n", data_get4);
        strcpy(data4, data_get4);
        cb_mqtt_4 = 1;
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success");

        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Write char failed, error status = %x", p_data->write.status);
        }else{
            ESP_LOGI(GATTC_TAG, "Write char success");
        }
        // start_scan();
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:%08x%04x",(bda[0] << 24) + (bda[1] << 16) + (bda[2] << 8) + bda[3],
                 (bda[4] << 8) + bda[5]);
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT:
        if (memcmp(p_data->disconnect.remote_bda, gl_profile_tab[PROFILE_D_APP_ID].remote_bda, 6) == 0){
            ESP_LOGI(GATTC_TAG, "device d disconnect");
            conn_device_d = false;
            get_service_d = false;
        }
        break;
    default:
        break;
    }
}

//handle for node 5
static void gattc_profile_e_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        break;
    case ESP_GATTC_CONNECT_EVT:
        break;
    case ESP_GATTC_OPEN_EVT:
        if (p_data->open.status != ESP_GATT_OK){
            //open failed, ignore the second device, connect the third device
            ESP_LOGE(GATTC_TAG, "connect device failed, status %d", p_data->open.status);
            conn_device_e = false;
            //start_scan();
            break;
        }
        memcpy(gl_profile_tab[PROFILE_E_APP_ID].remote_bda, p_data->open.remote_bda, 6);
        gl_profile_tab[PROFILE_E_APP_ID].conn_id = p_data->open.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"Config mtu failed");
        }
        ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            get_service_e = true;
            gl_profile_tab[PROFILE_E_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_E_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (get_service_e){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_E_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_E_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result_e = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result_e){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_E_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_E_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result_e,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result_e[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_E_APP_ID].char_handle = char_elem_result_e[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_E_APP_ID].remote_bda, char_elem_result_e[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result_e);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        start_scan();
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {

        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "reg notify failed, error status =%x", p_data->reg_for_notify.status);
            break;
        }
        uint16_t count = 0;
        //uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     gl_profile_tab[PROFILE_E_APP_ID].conn_id,
                                                                     ESP_GATT_DB_DESCRIPTOR,
                                                                     gl_profile_tab[PROFILE_E_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_E_APP_ID].service_end_handle,
                                                                     gl_profile_tab[PROFILE_E_APP_ID].char_handle,
                                                                     &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }
        if (count > 0){
            descr_elem_result_e = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result_e){
                ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
            }else{
                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                     gl_profile_tab[PROFILE_E_APP_ID].conn_id,
                                                                     p_data->reg_for_notify.handle,
                                                                     notify_descr_uuid,
                                                                     descr_elem_result_e,
                                                                     &count);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                }

                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }

                /* free descr_elem_result */
                free(descr_elem_result_e);
            }
        }
        else{
            ESP_LOGE(GATTC_TAG, "decsr not found");
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, Receive notify value:");
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);

        char data_get5[4] = {p_data->notify.value[0], p_data->notify.value[1], p_data->notify.value[2], p_data->notify.value[3]};
        printf("Data from node 5: %s\n", data_get5);
        strcpy(data5, data_get5);
        cb_mqtt_5 = 1;
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success");

        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Write char failed, error status = %x", p_data->write.status);
        }else{
            ESP_LOGI(GATTC_TAG, "Write char success");
        }
        // start_scan();
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:%08x%04x",(bda[0] << 24) + (bda[1] << 16) + (bda[2] << 8) + bda[3],
                 (bda[4] << 8) + bda[5]);
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT:
        if (memcmp(p_data->disconnect.remote_bda, gl_profile_tab[PROFILE_E_APP_ID].remote_bda, 6) == 0){
            ESP_LOGI(GATTC_TAG, "device e disconnect");
            conn_device_e = false;
            get_service_e = false;
        }
        break;
    default:
        break;
    }
}

//handle for node 6
static void gattc_profile_f_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        break;
    case ESP_GATTC_CONNECT_EVT:
        break;
    case ESP_GATTC_OPEN_EVT:
        if (p_data->open.status != ESP_GATT_OK){
            //open failed, ignore the second device, connect the third device
            ESP_LOGE(GATTC_TAG, "connect device failed, status %d", p_data->open.status);
            conn_device_f = false;
            //start_scan();
            break;
        }
        memcpy(gl_profile_tab[PROFILE_F_APP_ID].remote_bda, p_data->open.remote_bda, 6);
        gl_profile_tab[PROFILE_F_APP_ID].conn_id = p_data->open.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"Config mtu failed");
        }
        ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            get_service_f = true;
            gl_profile_tab[PROFILE_F_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_F_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (get_service_f){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_F_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_F_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result_f = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result_f){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_F_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_F_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result_f,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result_f[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_F_APP_ID].char_handle = char_elem_result_f[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_F_APP_ID].remote_bda, char_elem_result_f[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result_f);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        start_scan();
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {

        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "reg notify failed, error status =%x", p_data->reg_for_notify.status);
            break;
        }
        uint16_t count = 0;
        //uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     gl_profile_tab[PROFILE_F_APP_ID].conn_id,
                                                                     ESP_GATT_DB_DESCRIPTOR,
                                                                     gl_profile_tab[PROFILE_F_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_F_APP_ID].service_end_handle,
                                                                     gl_profile_tab[PROFILE_F_APP_ID].char_handle,
                                                                     &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }
        if (count > 0){
            descr_elem_result_f = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result_f){
                ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
            }else{
                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                     gl_profile_tab[PROFILE_F_APP_ID].conn_id,
                                                                     p_data->reg_for_notify.handle,
                                                                     notify_descr_uuid,
                                                                     descr_elem_result_f,
                                                                     &count);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                }

                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }

                /* free descr_elem_result */
                free(descr_elem_result_f);
            }
        }
        else{
            ESP_LOGE(GATTC_TAG, "decsr not found");
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, Receive notify value:");
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);

        char data_get6[4] = {p_data->notify.value[0], p_data->notify.value[1], p_data->notify.value[2], p_data->notify.value[3]};
        printf("Data from node 6: %s\n", data_get6);
        strcpy(data6, data_get6);
        cb_mqtt_6 = 1;
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success");

        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Write char failed, error status = %x", p_data->write.status);
        }else{
            ESP_LOGI(GATTC_TAG, "Write char success");
        }
        // start_scan();
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:%08x%04x",(bda[0] << 24) + (bda[1] << 16) + (bda[2] << 8) + bda[3],
                 (bda[4] << 8) + bda[5]);
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT:
        if (memcmp(p_data->disconnect.remote_bda, gl_profile_tab[PROFILE_F_APP_ID].remote_bda, 6) == 0){
            ESP_LOGI(GATTC_TAG, "device f disconnect");
            conn_device_f = false;
            get_service_f = false;
        }
        break;
    default:
        break;
    }
}

//handle for node 7
static void gattc_profile_g_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        break;
    case ESP_GATTC_CONNECT_EVT:
        break;
    case ESP_GATTC_OPEN_EVT:
        if (p_data->open.status != ESP_GATT_OK){
            //open failed, ignore the second device, connect the third device
            ESP_LOGE(GATTC_TAG, "connect device failed, status %d", p_data->open.status);
            conn_device_g = false;
            //start_scan();
            break;
        }
        memcpy(gl_profile_tab[PROFILE_G_APP_ID].remote_bda, p_data->open.remote_bda, 6);
        gl_profile_tab[PROFILE_G_APP_ID].conn_id = p_data->open.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"Config mtu failed");
        }
        ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            get_service_g = true;
            gl_profile_tab[PROFILE_G_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_G_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (get_service_g){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_G_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_G_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result_g = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result_g){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_G_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_G_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result_g,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result_g[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_G_APP_ID].char_handle = char_elem_result_g[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_G_APP_ID].remote_bda, char_elem_result_g[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result_g);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        start_scan();
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {

        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "reg notify failed, error status =%x", p_data->reg_for_notify.status);
            break;
        }
        uint16_t count = 0;
        //uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     gl_profile_tab[PROFILE_G_APP_ID].conn_id,
                                                                     ESP_GATT_DB_DESCRIPTOR,
                                                                     gl_profile_tab[PROFILE_G_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_G_APP_ID].service_end_handle,
                                                                     gl_profile_tab[PROFILE_G_APP_ID].char_handle,
                                                                     &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }
        if (count > 0){
            descr_elem_result_g = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result_g){
                ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
            }else{
                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                     gl_profile_tab[PROFILE_G_APP_ID].conn_id,
                                                                     p_data->reg_for_notify.handle,
                                                                     notify_descr_uuid,
                                                                     descr_elem_result_g,
                                                                     &count);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                }

                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }

                /* free descr_elem_result */
                free(descr_elem_result_g);
            }
        }
        else{
            ESP_LOGE(GATTC_TAG, "decsr not found");
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, Receive notify value:");
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);

        char data_get7[4] = {p_data->notify.value[0], p_data->notify.value[1], p_data->notify.value[2], p_data->notify.value[3]};
        printf("Data from node 7: %.*s\n", 4, data_get7);
        strcpy(data7, data_get7);
        cb_mqtt_7 = 1;
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success");

        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Write char failed, error status = %x", p_data->write.status);
        }else{
            ESP_LOGI(GATTC_TAG, "Write char success");
        }
        //start_scan();
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:%08x%04x",(bda[0] << 24) + (bda[1] << 16) + (bda[2] << 8) + bda[3],
                 (bda[4] << 8) + bda[5]);
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT:
        if (memcmp(p_data->disconnect.remote_bda, gl_profile_tab[PROFILE_G_APP_ID].remote_bda, 6) == 0){
            ESP_LOGI(GATTC_TAG, "device g disconnect");
            conn_device_g = false;
            get_service_g = false;
        }
        break;
    default:
        break;
    }
}

//handfle for gap event
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(GATTC_TAG, "Scan start success");
        }else{
            ESP_LOGE(GATTC_TAG, "Scan start failed");
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(GATTC_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "Searched Device Name Len %d", adv_name_len);
            esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
            ESP_LOGI(GATTC_TAG, "\n");
            if (Isconnecting){
                break;
            }
            if (conn_device_a && conn_device_b && conn_device_c && conn_device_d && conn_device_e 
                                                && conn_device_f  && conn_device_g && !stop_scan_done){
                stop_scan_done = true;
                esp_ble_gap_stop_scanning();
                ESP_LOGI(GATTC_TAG, "all devices are connected");
                break;
            }
            if (adv_name != NULL) {

                if (strlen(remote_device_name[0]) == adv_name_len && strncmp((char *)adv_name, remote_device_name[0], adv_name_len) == 0) {
                    if (conn_device_a == false) {
                        conn_device_a = true;
                        ESP_LOGI(GATTC_TAG, "Searched device %s", remote_device_name[0]);
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        Isconnecting = true;
                    }
                    break;
                }
                else if (strlen(remote_device_name[1]) == adv_name_len && strncmp((char *)adv_name, remote_device_name[1], adv_name_len) == 0) {
                    if (conn_device_b == false) {
                        conn_device_b = true;
                        ESP_LOGI(GATTC_TAG, "Searched device %s", remote_device_name[1]);
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_B_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        Isconnecting = true;
                    }
                    //break;
                }
                else if (strlen(remote_device_name[2]) == adv_name_len && strncmp((char *)adv_name, remote_device_name[2], adv_name_len) == 0) {
                    if (conn_device_c == false) {
                        conn_device_c = true;
                        ESP_LOGI(GATTC_TAG, "Searched device %s", remote_device_name[2]);
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_C_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        Isconnecting = true;
                    }
                    //break;
                }
                else if (strlen(remote_device_name[3]) == adv_name_len && strncmp((char *)adv_name, remote_device_name[3], adv_name_len) == 0) {
                    if (conn_device_d == false) {
                        conn_device_d = true;
                        ESP_LOGI(GATTC_TAG, "Searched device %s", remote_device_name[3]);
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_D_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        Isconnecting = true;
                    }
                    //break;
                }
                else if (strlen(remote_device_name[4]) == adv_name_len && strncmp((char *)adv_name, remote_device_name[4], adv_name_len) == 0) {
                    if (conn_device_e == false) {
                        conn_device_e = true;
                        ESP_LOGI(GATTC_TAG, "Searched device %s", remote_device_name[4]);
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_E_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        Isconnecting = true;
                    }
                    //break;
                }
                else if (strlen(remote_device_name[5]) == adv_name_len && strncmp((char *)adv_name, remote_device_name[5], adv_name_len) == 0) {
                    if (conn_device_f == false) {
                        conn_device_f = true;
                        ESP_LOGI(GATTC_TAG, "Searched device %s", remote_device_name[5]);
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_F_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        Isconnecting = true;
                    }
                    //break;
                }
                else if (strlen(remote_device_name[6]) == adv_name_len && strncmp((char *)adv_name, remote_device_name[6], adv_name_len) == 0) {
                    if (conn_device_g == false) {
                        conn_device_g = true;
                        ESP_LOGI(GATTC_TAG, "Searched device %s", remote_device_name[6]);
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_G_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        Isconnecting = true;
                    }
                    break;
                }

            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scan stop failed");
            break;
        }
        ESP_LOGI(GATTC_TAG, "Stop scan successfully");

        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Adv stop failed");
            break;
        }
        ESP_LOGI(GATTC_TAG, "Stop adv successfully");
        break;

    default:
        break;
    }
}

//handle for gattc event
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    //ESP_LOGI(GATTC_TAG, "EVT %d, gattc if %d, app_id %d", event, gattc_if, param->reg.app_id);

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

//set callback for mqtt handle
static void mqtt_cb_data(void* arg)
{
    for(;;) {
        if(flag_cb_mqtt)
        {
            ESP_LOGI(GATTC_TAG, "DATA: %.*s\r\n", len_cb_mqtt, data_cb_mqtt);
            //printf("DATA: %.*s\r\n", len_cb_mqtt, data_cb_mqtt);
            char set_up_node[7];
            strncpy(set_up_node, data_cb_mqtt+10, 6);
            flag_cb_mqtt = false;
            esp_ble_gattc_write_char( gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  6,
                                  (uint8_t *) set_up_node,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
            
            esp_ble_gattc_write_char( gl_profile_tab[PROFILE_B_APP_ID].gattc_if,
                                  gl_profile_tab[PROFILE_B_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                  6,
                                  (uint8_t *) set_up_node,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
            
            esp_ble_gattc_write_char( gl_profile_tab[PROFILE_C_APP_ID].gattc_if,
                                  gl_profile_tab[PROFILE_C_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_C_APP_ID].char_handle,
                                  6,
                                  (uint8_t *) set_up_node,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);

            esp_ble_gattc_write_char( gl_profile_tab[PROFILE_D_APP_ID].gattc_if,
                                  gl_profile_tab[PROFILE_D_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_D_APP_ID].char_handle,
                                  6,
                                  (uint8_t *) set_up_node,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
            esp_ble_gattc_write_char( gl_profile_tab[PROFILE_E_APP_ID].gattc_if,
                                  gl_profile_tab[PROFILE_E_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_E_APP_ID].char_handle,
                                  6,
                                  (uint8_t *) set_up_node,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
            esp_ble_gattc_write_char( gl_profile_tab[PROFILE_F_APP_ID].gattc_if,
                                  gl_profile_tab[PROFILE_F_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_F_APP_ID].char_handle,
                                  6,
                                  (uint8_t *) set_up_node,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
            esp_ble_gattc_write_char( gl_profile_tab[PROFILE_G_APP_ID].gattc_if,
                                  gl_profile_tab[PROFILE_G_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_G_APP_ID].char_handle,
                                  6,
                                  (uint8_t *) set_up_node,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        }
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}

//main function
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    //start BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gap register error, error code = %x", ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "gattc register error, error code = %x", ret);
        return;
    }

    app_config(); //start wifi
    vTaskDelay(10 * 1000/portTICK_PERIOD_MS);
    mqtt_app_start(); //start mqtt
    xTaskCreate(mqtt_cb_data, "mqtt_cb_data", 4096, NULL, 10, NULL); //callback went receive set_up
    vTaskDelay(10 * 1000/portTICK_PERIOD_MS);
    
    //handle event for nodes
    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_B_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_C_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_D_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_E_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_F_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
        return;
    }
    
    ret = esp_ble_gattc_app_register(PROFILE_G_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatt_set_local_mtu(200);
    if (ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", ret);
    }
    
}
