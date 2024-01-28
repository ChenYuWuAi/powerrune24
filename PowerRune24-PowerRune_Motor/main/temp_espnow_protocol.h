#pragma once
#ifndef _ESP_NOW_PROTOCOL_H_
#define _ESP_NOW_PROTOCOL_H_

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "firmware.h"
#include "PowerRune_Events.h"
#include "motor_ctrl.h"

// #ifdef CONFIG_POWER_RUNE_TYPE == 1 // 主设备
// #define ESPNOW_QUEUE_SIZE 6
// #elif ((CONFIG_POWER_RUNE_TYPE == 2) || (CONFIG_POWER_RUNE_TYPE == 0))// 从设备
#define ESPNOW_QUEUE_SIZE 1
// #endif

#define ESP_NOW_DATA_LEN 200
static const char *TAG = "ESPNOW_EVENT";
esp_event_loop_handle_t loop_ESPNOW;
esp_event_loop_args_t loop_ESPNOW_args = {
    .queue_size = 4,
    .task_name = "loop_ESPNOW",
    .task_priority = 1,
    .task_stack_size = 2048,
    .task_core_id = 0,
};

// espnow 数据包
typedef struct
{
    // 大符事件数据头 0x24DF 4Bytes
    uint16_t event_header = 0x24DF;
    // 包ID 4Bytes
    uint16_t pack_id = 0;
    // CRC16 校验 2Bytes
    uint8_t crc16 = 0;
    // 事件根基 eventbase 4Bytes char[4]
    char event_base[4] = {0};
    // 事件数据长度 2Bytes
    uint8_t event_data_len = 0;
    // 事件ID 1Bytes
    uint8_t event_id = 0;
    // 事件数据
    uint8_t event_data[ESP_NOW_DATA_LEN - 17] = {0};
}espnow_DATA_pack_t;

// espnow 大符ACK_OK包 
typedef struct
{
    // 大符事件数据头 0x9439 4Bytes
    uint16_t event_header = 0x9439;
    // 包ID 4Bytes
    uint32_t pack_id = 0;
}ACK_OK_pack_t;

// espnow 大符ACK_FAIL包
typedef struct
{
    // 大符事件数据头 0xF2A6 4Bytes
    uint16_t event_header = 0xF2A6;
    // 包ID 4Bytes
    uint32_t pack_id = 0;
}ACK_FAIL_pack_t;

enum PacketType {
    DATA,
    ACK_OK,
    ACK_FAIL
};

// espnow MAC地址, 使用静态多态对主控(接收6个设备, 5个ESP32S3, 1个ESP32C3)和从控(6个分设备)(发送)的MAC地址
// #ifdef CONFIG_POWER_RUNE_TYPE == 1 // 主设备
// static uint8_t mac_addr[6][ESP_NOW_MAC_ADDR_LEN] = {0};
// #elif ((CONFIG_POWER_RUNE_TYPE == 2) || (CONFIG_POWER_RUNE_TYPE == 0))// 从设备
static uint8_t mac_addr[ESP_NOW_ETH_ALEN] = {0};
// #endif

static QueueHandle_t espnow_queue = NULL;

static void rx_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data){
        espnow_DATA_pack_t* espnow_data_pack = (espnow_DATA_pack_t *)handler_args;
        ESP_ERROR_CHECK(esp_event_post_to(loop_PRM, event_base, ESP_EVENT_ANY_ID, espnow_data_pack->event_data, sizeof(espnow_data_pack->event_data), portMAX_DELAY));     
}

class ESPNowProtocol {
private:
    //??

public:
    static esp_err_t parse_data(const uint8_t *data, int len){
        if (len < 2) {
            ESP_LOGI(TAG,"parse data fail len < 2");
            return ESP_FAIL;
        }
        uint16_t event_header = (data[0] << 8) | data[1];

        if(event_header == 0x24DF){
            // espnow_DATA_pack_t
            espnow_DATA_pack_t espnow_data_pack;
            memset(&espnow_data_pack, 0, sizeof(espnow_DATA_pack_t));
            espnow_data_pack.event_header = (data[0] << 8) | data[1];
            espnow_data_pack.pack_id = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
            espnow_data_pack.crc16 = (data[6] << 8) | data[7];
            char event_base_str[4] = {data[8], data[9], data[10], data[11]};
            memcpy(espnow_data_pack.event_base, event_base_str, 4);
            espnow_data_pack.event_data_len = (data[12] << 8) | data[13];
            espnow_data_pack.event_id = data[14];
            memcpy(espnow_data_pack.event_data, data + 15, len - 15);
            uint16_t crc16 = esp_crc16_le(UINT16_MAX, (uint8_t *)&espnow_data_pack, len - 2);
            if (crc16 != espnow_data_pack.crc16) {
                ESP_LOGE(TAG, "CRC16 check failed, drop packet");
                return ESP_FAIL;
            }
            xQueueSend(espnow_queue, &espnow_data_pack, portMAX_DELAY);
        }
        else if(event_header == 0x9439){
            // 这是一个ACK_OK_pack_t包, 
            ACK_OK_pack_t ack_ok_pack;
            memset(&ack_ok_pack, 0, sizeof(ACK_OK_pack_t));
            ack_ok_pack.event_header = (data[0] << 8) | data[1];
            ack_ok_pack.pack_id = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
            xQueueSend(espnow_queue, &ack_ok_pack, portMAX_DELAY);
        }
        else if(event_header == 0xF2A6){
            // 这是一个ACK_FAIL_pack_t包
            ACK_FAIL_pack_t ack_fail_pack;
            memset(&ack_fail_pack, 0, sizeof(ACK_FAIL_pack_t));
            ack_fail_pack.event_header = (data[0] << 8) | data[1];
            ack_fail_pack.pack_id = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
            xQueueSend(espnow_queue, &ack_fail_pack, portMAX_DELAY);
        }
        else{
            return ESP_FAIL; 
        }
        
    }

    static esp_err_t get_decoded_data(espnow_DATA_pack_t *data) {
        if (xQueueReceive(espnow_queue, data, portMAX_DELAY) == pdTRUE) {
            return ESP_OK;
        } else {
            return ESP_FAIL;
        }
    }

    static esp_now_recv_cb_t rx_callback(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int len) {
        // 待添加ESP_FAIL处理
        // 中断回调函数参数参考: const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len
        const uint8_t *mac_addr = esp_now_info->src_addr;

        ESP_LOGI(TAG, "Received from: " MACSTR, MAC2STR(mac_addr));

        esp_err_t err = ESPNowProtocol::parse_data(data, len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to parse data: %d", err);
        }
        // 从数据包提取eventbase, eventid, data, data输入到void *handler_args, 根据eventbase, eventid来post event, 调用rx_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data)函数
        espnow_DATA_pack_t* espnow_data_pack;
        memset(&espnow_data_pack, 0, sizeof(espnow_DATA_pack_t));
        get_decoded_data(espnow_data_pack);
        ESP_ERROR_CHECK(esp_event_handler_register_with(loop_ESPNOW, ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, rx_event_handler, espnow_data_pack));
        return ESP_OK;
    }

    static esp_err_t prepare_data(void *packet, PacketType type, uint8_t *output, size_t *output_len) {
        if (type == DATA) {
            espnow_DATA_pack_t *data_packet = (espnow_DATA_pack_t *)packet;
            memcpy(output, data_packet, sizeof(espnow_DATA_pack_t));
            uint16_t crc16 = esp_crc16_le(UINT16_MAX, (uint8_t *)data_packet, sizeof(espnow_DATA_pack_t) - 2);
            // 将CRC16校验和添加到数据包uint8_t crc16 = 0;
            data_packet->crc16 = crc16;
            *output_len = sizeof(espnow_DATA_pack_t);
        } else if (type == ACK_OK) {
            ACK_OK_pack_t *ack_ok_packet = (ACK_OK_pack_t *)packet;
            memcpy(output, ack_ok_packet, sizeof(ACK_OK_pack_t));
            *output_len = sizeof(ACK_OK_pack_t);
        } else if (type == ACK_FAIL) {
            ACK_FAIL_pack_t *ack_fail_packet = (ACK_FAIL_pack_t *)packet;
            memcpy(output, ack_fail_packet, sizeof(ACK_FAIL_pack_t));
            *output_len = sizeof(ACK_FAIL_pack_t);
        } else {
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    static esp_err_t send_data(void *packet, PacketType type) {
        uint8_t output[ESP_NOW_DATA_LEN];
        size_t output_len;

        esp_err_t err = prepare_data(packet, type, output, &output_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to prepare data: %d", err);
            return err;
        }

        if (xQueueSend(espnow_queue, output, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to send data to queue");
            return ESP_FAIL;
        }

        return ESP_OK;
    }

    static esp_err_t tx_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
        // 待 需要添加内容, 暂时没想好
        // const uint8_t *mac_addr, esp_now_send_status_t status
        if (status == ESP_NOW_SEND_SUCCESS) {
            ESP_LOGI(TAG, "Send data successfully");
        } else {
            ESP_LOGE(TAG, "Send data failed");
        }
        return ESP_OK;
    }
    
    static void tx_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data){
        if (event_base == OTA_EVENTS && event_id == OTA_COMPLETE_EVENT) {
            espnow_DATA_pack_t data_packet;
            memset(&data_packet, 0, sizeof(data_packet));
            data_packet.event_header = 0x24DF;
            data_packet.pack_id = 0; 
            strncpy(data_packet.event_base, "OTA", sizeof(data_packet.event_base));
            data_packet.event_data_len = 0; 
            data_packet.event_id = OTA_COMPLETE_EVENT;
            esp_err_t err = send_data(&data_packet, DATA);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send data: %d", err);
            }
        }
    }

    ESPNowProtocol() {

        //数据包ID均未配置!!!
        ESP_ERROR_CHECK(esp_now_init());
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_ERROR_CHECK(esp_event_loop_create(&loop_ESPNOW_args, &loop_ESPNOW));

        assert(espnow_queue != NULL);
        espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_DATA_pack_t));

        ESP_ERROR_CHECK(esp_event_handler_register_with(loop_PRM, ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, tx_event_handler, NULL));

        ESP_ERROR_CHECK(esp_now_register_recv_cb(rx_callback));
        // ESP_ERROR_CHECK(esp_now_register_send_cb(tx_callback));

        

    }
};



#endif