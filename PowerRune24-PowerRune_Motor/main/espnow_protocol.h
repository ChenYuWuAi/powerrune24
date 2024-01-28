/**
 * @file espnow_protocol.h
 * @brief ESP-NOW protocol header file
*/

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
#include "queue.h"

#include "firmware.h"

#define TAG "ESPNOW_PROTOCOL_SETUP"

#define ESP_NOW_DATA_LEN 150
#define ESP_NOW_MAC_ADDR_LEN 6

static QueueHandle_t espnow_queue = NULL;
#ifdef CONFIG_POWER_RUNE_TYPE == 1 // 主设备
#define ESPNOW_QUEUE_SIZE 6
#elif ((CONFIG_POWER_RUNE_TYPE == 2) || (CONFIG_POWER_RUNE_TYPE == 0))// 从设备
#define ESPNOW_QUEUE_SIZE 1
#endif

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
    uint8_t event_data[ESP_NOW_DATA_LEN] = {0};
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

// espnow是否组网
typedef enum
{
    ESPNOW_NOT_INIT,
    ESPNOW_INIT,
}espnow_init_t;

// espnow MAC地址, 使用静态多态对主控(接收6个设备, 5个ESP32S3, 1个ESP32C3)和从控(6个分设备)(发送)的MAC地址
typedef struct
{
#ifdef CONFIG_POWER_RUNE_TYPE == 1 // 主设备
// 二维数组是否合适?
    uint8_t mac_addr[6][ESP_NOW_MAC_ADDR_LEN] = {0};
#elif ((CONFIG_POWER_RUNE_TYPE == 2) || (CONFIG_POWER_RUNE_TYPE == 0))// 从设备
    uint8_t mac_addr[ESP_NOW_MAC_ADDR_LEN] = {0};
#endif
}peer_info_t;


class ESPNOW_PROTOCOL{
private:
    ~ESPNOW_PROTOCOL();

public:
    ESPNOW_PROTOCOL() {
        
    };



    static esp_err_t rx_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len){
        if (len < 2) {
            return ESP_FAIL;
        }
        // 从数据中读取event_header
        uint16_t event_header = (data[0] << 8) | data[1];
        // 根据event_header的值来区分不同的包
        switch (event_header) {
        case 0x24DF:
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
            break;
        case 0x9439:
            // 这是一个ACK_OK_pack_t包, 使用移位运算将data转换为ACK_OK_pack_t
            ACK_OK_pack_t ack_ok_pack;
            memset(&ack_ok_pack, 0, sizeof(ACK_OK_pack_t));
            ack_ok_pack.event_header = (data[0] << 8) | data[1];
            ack_ok_pack.pack_id = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];

            break;
        case 0xF2A6:
            // 这是一个ACK_FAIL_pack_t包
            ACK_FAIL_pack_t ack_fail_pack;
            memset(&ack_fail_pack, 0, sizeof(ACK_FAIL_pack_t));
            ack_fail_pack.event_header = (data[0] << 8) | data[1];
            ack_fail_pack.pack_id = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
            break;
        default:
            
            return ESP_FAIL;
        }

};


#endif