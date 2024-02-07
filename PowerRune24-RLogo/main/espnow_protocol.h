/**
 * @file espnow_protocol.h
 * @brief ESP-NOW协议类，用于ESP-NOW通信
 * @note 本文件存放ESP-NOW协议类的声明，Wifi硬件初始化，esp-now的发送和接收回调函数，事件处理
 */
#pragma once

#ifndef _ESP_NOW_PROTOCOL_H_
#define _ESP_NOW_PROTOCOL_H_

// ESP Common
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_err.h"
#include "esp_crc.h"
// PowerRune Common
#include "firmware.h"
#include "PowerRune_Events.h"

// espnow 数据包队列大小
#ifdef CONFIG_POWERRUNE_TYPE == 1 // 主设备
#define ESPNOW_QUEUE_SIZE 6
#elif ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
#define ESPNOW_QUEUE_SIZE 1
#endif

// 包头
#define POWERRUNE_DATA_HEADER 0x24DF
#define POWERRUNE_ACK_OK_HEADER 0x9439
#define POWERRUNE_ACK_FAIL_HEADER 0xF2A6

// 事件数据长度
#define ESP_NOW_DATA_LEN 200
// 超时重传次数
#define RETRY_COUNT 3
static const char *TAG_MESSAGER = "Messager";

// espnow 数据包
typedef struct
{
    // 大符事件数据头 POWERRUNE_DATA_HEADER 2Bytes
    uint16_t header = POWERRUNE_DATA_HEADER;
    // 包ID 2Bytes
    uint16_t pack_id = 0;
    // CRC16 校验事件数据 2Bytes
    uint16_t crc16 = 0;
    // 事件根基 eventbase 4Bytes char[4]
    char event_base[4] = {0};
    // 事件数据长度 2Bytes
    uint16_t event_data_len = 0;
    // 事件ID 1Bytes
    uint8_t event_id = 0;
    // 事件数据
    uint8_t event_data[ESP_NOW_DATA_LEN] = {0};

    uint8_t dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
} espnow_DATA_pack_t;                   // Size: 2 + 2 + 2 + 4 + 2 + 1 + 200 + 6 = 219

// espnow 大符ACK_OK包，ACK的ID与DATA的ID相同
typedef struct
{
    // 大符事件数据头 0x9439 4Bytes
    uint16_t header = POWERRUNE_ACK_OK_HEADER;
    // 包ID 2Bytes
    uint16_t pack_id = 0;

    uint8_t dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
} ACK_OK_pack_t;                        // Size: 4 + 2 + 6 = 12Bytes

// espnow 大符ACK_FAIL包，ACK的ID与DATA的ID相同
typedef struct
{
    // 大符事件数据头 0xF2A6 4Bytes
    uint16_t header = POWERRUNE_ACK_FAIL_HEADER;
    // 包ID 2Bytes
    uint16_t pack_id = 0;

    uint8_t dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
} ACK_FAIL_pack_t;                      // Size: 4 + 2 + 6 = 12Bytes

enum PacketType
{
    DATA,
    ACK_OK,
    ACK_FAIL,
};

enum PowerRune_Devices
{
    ARMOUR1,
    ARMOUR2,
    ARMOUR3,
    ARMOUR4,
    ARMOUR5,
    MOTOR,
    RLOGO,
};

/**
 * @brief ESP-NOW协议类
 * @note 用于ESP-NOW通信
 发送路径: 事件循环 -> tx_event_handler -> send_data
 接收路径: rx_callback -> parse_data -> rx_event_handler
                                    -> ACK_OK/ACK_FAIL
 状态变量: send_state，其中对外可见的是SEND_ACK_OK_BIT和SEND_ACK_FAIL_BIT
*/
class ESPNowProtocol
{
private: // espnow 数据包队列
// espnow MAC地址, 使用静态多态，对主控(收发6个设备, 5个ESP32S3[0:4], 1个ESP32C3[5])和从控(6个分设备发送)的MAC地址
#ifdef CONFIG_POWERRUNE_TYPE == 1 // 主设备
    static uint8_t mac_addr[6][ESP_NOW_ETH_ALEN];
    // 包ID，一方的TX_ID随包发送，原则上应该比对方的RX_ID大1，否则说明有包丢失
    static uint16_t packet_tx_id[6];
    static uint16_t packet_rx_id[6];
#elif ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
    static uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    // 包ID
    static uint16_t packet_tx_id;
    static uint16_t packet_rx_id;
#endif
    // #endif
    static QueueHandle_t espnow_rx_queue;
    // tx mutex
    static SemaphoreHandle_t tx_semaphore;

    typedef struct
    {
        uint8_t src_MAC[ESP_NOW_ETH_ALEN];
        uint8_t *data;
        uint8_t len;
    } received_struct;

    // peer list是否成功建立
    static bool peer_list_established;
    /**
     * @brief 打印espnow数据包
     * @param data espnow数据包
     */
    static void log_packet(const espnow_DATA_pack_t *data)
    {
        ESP_LOGD(TAG_MESSAGER, "header: 0x%04X", data->header);
        ESP_LOGD(TAG_MESSAGER, "pack_id: 0x%08X", data->pack_id);
        ESP_LOGD(TAG_MESSAGER, "event_base: %s", data->event_base);
        ESP_LOGD(TAG_MESSAGER, "event_id: 0x%02X", data->event_id);
        ESP_LOGD(TAG_MESSAGER, "event_data_len: %d", data->event_data_len);
    }

    static void log_packet(const ACK_OK_pack_t *data)
    {
        ESP_LOGD(TAG_MESSAGER, "header: 0x%04X", data->header);
        ESP_LOGD(TAG_MESSAGER, "pack_id: 0x%08X", data->pack_id);
    }

    static void log_packet(const ACK_FAIL_pack_t *data)
    {
        ESP_LOGD(TAG_MESSAGER, "header: 0x%04X", data->header);
        ESP_LOGD(TAG_MESSAGER, "pack_id: 0x%08X", data->pack_id);
    }

    static const int SEND_COMPLETE_BIT = BIT3;    // 已发送，正在等待ACK，内部使用
    static const int SEND_FAIL_BIT = BIT4;        // 发送失败（未传出），内部使用
    static const int SEND_ACK_PENDING_BIT = BIT6; // 已发送，等待ACK，内部使用

#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
    static uint8_t mac_to_address(uint8_t *mac)
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            if (memcmp(mac, mac_addr[i], ESP_NOW_ETH_ALEN) == 0)
            {
                return i;
            }
        }
        return 0;
    }
#endif

public:
    // 自身MAC
    static uint8_t self_mac[ESP_NOW_ETH_ALEN];
    // 广播地址
    static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN];
    // 发送数据EventBits
    static EventGroupHandle_t send_state;
    // 先置位BUSY，然后发送数据，发送完成置位COMPLETE，等待ACK，收到ACK后置位OK，超时重传，清除BUSY和COMPLETE
    static const int SEND_ACK_OK_BIT = BIT0;      // 已发送，收到ACK_OK包，外部可读
    static const int SEND_ACK_FAIL_BIT = BIT1;    // 已发送，收到ACK_FAIL包，外部可读
    static const int SEND_BUSY = BIT2;            // 正在发送，外部可读，内部可用于判断是否接受ACK包
    static const int SEND_ACK_TIMEOUT_BIT = BIT5; // 已发送，等待ACK超时

    static void
    rx_callback(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
    {
        // 待添加ESP_FAIL处理
        // 中断回调函数参数参考: const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len
        const uint8_t *mac_addr = esp_now_info->src_addr;

        if (mac_addr == NULL || data == NULL || len <= 0)
        {
            ESP_LOGE(TAG_MESSAGER, "Receive cb arg error");
            return;
        }

        // 比较MAC地址，如果不是自己的MAC地址或者广播地址，直接丢弃
        if (memcmp(esp_now_info->des_addr, self_mac, ESP_NOW_ETH_ALEN) != 0 || memcmp(esp_now_info->des_addr, broadcast_mac, ESP_NOW_ETH_ALEN) != 0)
        {
            return;
        }

        // 存入数据
        received_struct *received_data = (received_struct *)malloc(sizeof(received_struct));
        if (received_data == NULL)
        {
            ESP_LOGE(TAG_MESSAGER, "malloc received_data failed");
            return;
        }
        memcpy(received_data->src_MAC, mac_addr, ESP_NOW_ETH_ALEN);
        received_data->data = (uint8_t *)malloc(len);
        memcpy(received_data->data, data, len);
        received_data->len = len;
        // 发送到espnow_rx_queue
        xQueueSendFromISR(espnow_rx_queue, received_data, pdFALSE);
    }

    /**
     * @brief 解析espnow数据包
     * @param pvParameter 任务参数
     */
    static void parse_data_task(void *pvParameter)
    {

        received_struct *received_data;
        uint8_t *data = received_data->data;

        while (1)
        {
            if (xQueueReceive(espnow_rx_queue, received_data, portMAX_DELAY) != pdTRUE)
            {
                ESP_LOGE(TAG_MESSAGER, "Failed to receive data from espnow_rx_queue");
                continue;
            }

            ESP_LOGD(TAG_MESSAGER, "Received %i bytes from " MACSTR, received_data->len, MAC2STR(received_data->src_MAC));

            uint16_t header = (data[0] << 8) | data[1];
            // 解析大符事件数据包
            if (header == POWERRUNE_DATA_HEADER)
            {
                espnow_DATA_pack_t espnow_data_pack = {0};
                // 按字节解析
                espnow_data_pack.header = header;
                espnow_data_pack.pack_id = (data[2] << 24) |
                                           (data[3] << 16) |
                                           (data[4] << 8) | data[5];

// ID校验
#ifdef CONFIG_POWERRUNE_TYPE == 1 // 主设备
                // 已发送的包ID小于等于接收的包ID，说明这个包已经无效
                if (espnow_data_pack.pack_id <= packet_rx_id[mac_to_address(received_data->src_MAC)])
                {
                    ESP_LOGE(TAG_MESSAGER, "packet %i ID check failed", espnow_data_pack.pack_id);
                    // 直接丢包，释放内存
                    free(received_data->data);
                    continue;
                }
                else
                {
                    packet_rx_id[mac_to_address(received_data->src_MAC)] = espnow_data_pack.pack_id;
                }
#endif
#if ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
                // 已发送的包ID小于等于接收的包ID，说明这个包已经无效
                if (espnow_data_pack.pack_id <= packet_rx_id)
                {
                    ESP_LOGE(TAG_MESSAGER, "DATA packet %i ID check failed", espnow_data_pack.pack_id);
                    // 直接丢包，释放内存
                    free(received_data->data);
                    continue;
                }
                else
                {
                    packet_rx_id = espnow_data_pack.pack_id;
                }
#endif

                // 拷贝CRC，原包内CRC16保持为0
                uint16_t crc16_pack = (data[6] << 8) | data[7];
                memcpy(espnow_data_pack.event_base, data + 6, 4);
                espnow_data_pack.event_data_len = (data[10] << 8) | data[11];
                espnow_data_pack.event_id = data[12];
                memcpy(espnow_data_pack.event_data, data + 13, espnow_data_pack.event_data_len);
                // CRC校验
                uint16_t crc16 = esp_crc16_le(UINT16_MAX, (uint8_t *)&espnow_data_pack, sizeof(espnow_DATA_pack_t) - 2);
                log_packet(&espnow_data_pack);

                if (crc16 != crc16_pack)
                {
                    ESP_LOGE(TAG_MESSAGER, "DATA packet %i CRC16 check failed", espnow_data_pack.pack_id);
                    // 发送ACK_FAIL，然后丢包
                    send_ACK(espnow_data_pack.pack_id, received_data->src_MAC, ACK_FAIL);
                    // 释放内存
                    free(received_data->data);
                    continue;
                }
                else
                {
                    // 发送ACK_OK
                    send_ACK(espnow_data_pack.pack_id, received_data->src_MAC, ACK_OK);

                    // 发送到事件循环
                    esp_event_post_to(pr_events_loop_handle, espnow_data_pack.event_base,
                                      espnow_data_pack.event_id, espnow_data_pack.event_data,
                                      espnow_data_pack.event_data_len,
                                      CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);
                }
            }
            // 正在等待ACK的包
            else if (header == POWERRUNE_ACK_OK_HEADER && (xEventGroupGetBits(send_state) & SEND_ACK_PENDING_BIT))
            {
                // 这是一个ACK_OK包,
                ACK_OK_pack_t ack_ok_pack = {0};
                ack_ok_pack.header = header;
                ack_ok_pack.pack_id = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
                log_packet(&ack_ok_pack);
                // ID校验，丢弃过期ACK包
#ifdef CONFIG_POWERRUNE_TYPE == 1 // 主设备
                if (ack_ok_pack.pack_id <= packet_tx_id[mac_to_address(received_data->src_MAC)])
                {
                    ESP_LOGE(TAG_MESSAGER, "ACK_OK packet %i ID check failed", ack_ok_pack.pack_id);
                    // 释放内存
                    free(received_data->data);
                    continue;
                }
#endif
#if ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
                if (ack_ok_pack.pack_id <= packet_tx_id)
                {
                    ESP_LOGE(TAG_MESSAGER, "ACK_OK packet %i ID check failed", ack_ok_pack.pack_id);
                    // 释放内存
                    free(received_data->data);
                    continue;
                }
#endif
                // 发送ACK_OK
                xEventGroupSetBits(send_state, SEND_ACK_OK_BIT);
                // 释放内存
                free(received_data->data);
            }
            else if (header == POWERRUNE_ACK_FAIL_HEADER && (xEventGroupGetBits(send_state) & SEND_ACK_PENDING_BIT))
            {
                // 这是一个ACK_FAIL包
                ACK_FAIL_pack_t ack_fail_pack = {0};
                ack_fail_pack.header = header;
                ack_fail_pack.pack_id = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
                log_packet(&ack_fail_pack);
                // ID校验，丢弃过期ACK包
#ifdef CONFIG_POWERRUNE_TYPE == 1 // 主设备
                if (ack_fail_pack.pack_id <= packet_tx_id[mac_to_address(received_data->src_MAC)])
                {
                    ESP_LOGE(TAG_MESSAGER, "ACK_FAIL packet %i ID check failed", ack_fail_pack.pack_id);
                    // 释放内存
                    free(received_data->data);
                    continue;
                }
#endif
#if ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
                if (ack_fail_pack.pack_id <= packet_tx_id)
                {
                    ESP_LOGE(TAG_MESSAGER, "ACK_FAIL packet %i ID check failed", ack_fail_pack.pack_id);
                    // 释放内存
                    free(received_data->data);
                    continue;
                }
#endif
                // 发送ACK_FAIL
                xEventGroupSetBits(send_state, SEND_ACK_FAIL_BIT);
                // 释放内存
                free(received_data->data);
            }
            else
            {
                // 收到未知包，丢弃
                ESP_LOGE(TAG_MESSAGER, "Unknown packet header: 0x%04X", header);
                // 释放内存
                free(received_data->data);
                continue;
            }
        }
    }

    static esp_err_t send_data(uint16_t packet_tx_id, uint8_t *dest_mac, esp_event_base_t event_base, int32_t *event_id, void *data, size_t data_len, uint8_t wait_ack = 1)
    {
        // 获取互斥锁
        if (xSemaphoreTake(tx_semaphore, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG_MESSAGER, "Failed to take tx_semaphore");
            return ESP_FAIL;
        }
        // 编码包数据
        espnow_DATA_pack_t *packet = (espnow_DATA_pack_t *)malloc(sizeof(espnow_DATA_pack_t));
        packet->header = POWERRUNE_DATA_HEADER;
        packet->pack_id = packet_tx_id;
        packet->event_data_len = data_len;
        // event_data置零
        memset(packet->event_data, 0, ESP_NOW_DATA_LEN);
        memcpy(packet->event_data, data, data_len);
        memcpy(packet->event_base, event_base, sizeof(event_base));
        packet->event_id = *event_id;
        // CRC置零后进行运算
        packet->crc16 = 0;
        // 将CRC16校验和添加到数据包
        packet->crc16 = esp_crc16_le(UINT16_MAX, (uint8_t *)packet, sizeof(espnow_DATA_pack_t) - 6); // 去掉dest_mac
        memcpy(packet->dest_mac, dest_mac, ESP_NOW_ETH_ALEN);

        // 调用发送函数，含超时重传
        for (uint8_t trial = 0; trial < RETRY_COUNT; trial++)
        {
            esp_err_t err = esp_now_send(dest_mac, (uint8_t *)packet, sizeof(espnow_DATA_pack_t) - 6);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG_MESSAGER, "packet %i send error %s, trial %i", packet_tx_id, esp_err_to_name(err), trial);
                continue;
            }
            // 等待发送成功或失败
            xEventGroupWaitBits(send_state, SEND_COMPLETE_BIT | SEND_FAIL_BIT, pdFALSE, pdFALSE, CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);

            if (xEventGroupGetBits(send_state) & SEND_COMPLETE_BIT)
            {
                ESP_LOGD(TAG_MESSAGER, "packet %i send success, waiting for ACK", packet_tx_id);
                trial = 0; // 重置重传次数
                xEventGroupClearBits(send_state, SEND_COMPLETE_BIT);
            }
            else if (xEventGroupGetBits(send_state) & SEND_FAIL_BIT)
            {
                ESP_LOGE(TAG_MESSAGER, "packet %i send error in tx callback, trial %i", packet_tx_id, trial);
                xEventGroupClearBits(send_state, SEND_FAIL_BIT);
                continue;
            }
            else
            {
                ESP_LOGE(TAG_MESSAGER, "packet %i send timeout, trial %i", packet_tx_id, trial);
                xEventGroupClearBits(send_state, SEND_COMPLETE_BIT | SEND_FAIL_BIT);
                continue;
            }

            if (wait_ack == 0)
            {
                // 释放互斥锁
                xSemaphoreGive(tx_semaphore);
                return ESP_OK;
            }

            // 置位使能ACK等待
            xEventGroupSetBits(send_state, SEND_ACK_PENDING_BIT);

            // 等待ACK_OK或ACK_FAIL
            xEventGroupWaitBits(send_state, SEND_ACK_OK_BIT | SEND_ACK_FAIL_BIT, pdFALSE, pdFALSE, CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);
            xEventGroupClearBits(send_state, SEND_ACK_PENDING_BIT);

            if (xEventGroupGetBits(send_state) & SEND_ACK_OK_BIT) // 收到ACK_OK
            {
                ESP_LOGD(TAG_MESSAGER, "packet %i ACK_OK received", packet_tx_id);
                xEventGroupClearBits(send_state, SEND_COMPLETE_BIT);
                free(packet);
                // 释放互斥锁
                xSemaphoreGive(tx_semaphore);
                return ESP_OK;
            }
            else
            {
                ESP_LOGE(TAG_MESSAGER, "packet %i ACK_FAIL received", packet_tx_id);
                xEventGroupClearBits(send_state, SEND_COMPLETE_BIT);
                // 重传
                continue;
            }
        }
        // 超时重传次数用完
        xEventGroupSetBits(send_state, SEND_ACK_TIMEOUT_BIT);
        xEventGroupClearBits(send_state, SEND_COMPLETE_BIT | SEND_ACK_OK_BIT | SEND_ACK_FAIL_BIT);
        ESP_LOGE(TAG_MESSAGER, "packet %i send failed", packet_tx_id);
        free(packet);
        // 释放互斥锁
        xSemaphoreGive(tx_semaphore);
        return ESP_FAIL;
    }

    static esp_err_t send_ACK(uint16_t packet_tx_id, uint8_t *dest_mac, PacketType type)
    {
        // 获取互斥锁
        if (xSemaphoreTake(tx_semaphore, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG_MESSAGER, "Failed to take tx_semaphore");
            return ESP_FAIL;
        }
        ACK_OK_pack_t *packet = (ACK_OK_pack_t *)malloc(sizeof(ACK_OK_pack_t));
        packet->header = (type == ACK_OK) ? POWERRUNE_ACK_OK_HEADER : POWERRUNE_ACK_FAIL_HEADER;
        packet->pack_id = packet_tx_id;
        memcpy(packet->dest_mac, dest_mac, ESP_NOW_ETH_ALEN);
        // send
        esp_err_t err = esp_now_send(packet->dest_mac, (uint8_t *)packet, sizeof(ACK_OK_pack_t) - 6);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_MESSAGER, "ACK %i packet %i send error %s", type, packet_tx_id, esp_err_to_name(err));
            free(packet);
            return err;
        }
        // 等待发送成功或失败
        xEventGroupWaitBits(send_state, SEND_COMPLETE_BIT | SEND_FAIL_BIT, pdFALSE, pdFALSE, CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);
        if (xEventGroupGetBits(send_state) & SEND_COMPLETE_BIT)
        {
            ESP_LOGD(TAG_MESSAGER, "ACK %i packet %i send success", type, packet_tx_id);
            xEventGroupClearBits(send_state, SEND_COMPLETE_BIT);
        }
        else if (xEventGroupGetBits(send_state) & SEND_FAIL_BIT)
        {
            ESP_LOGE(TAG_MESSAGER, "ACK %i packet %i send error in tx callback", type, packet_tx_id);
            xEventGroupClearBits(send_state, SEND_FAIL_BIT);
            free(packet);
            return ESP_FAIL;
        }
        else
        {
            ESP_LOGE(TAG_MESSAGER, "ACK %i packet %i send timeout", type, packet_tx_id);
            xEventGroupClearBits(send_state, SEND_COMPLETE_BIT | SEND_FAIL_BIT);
            free(packet);
            return ESP_FAIL;
        }
        free(packet);
        // 释放互斥锁
        xSemaphoreGive(tx_semaphore);
        return ESP_OK;
    }

    static void tx_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        if (status == ESP_NOW_SEND_SUCCESS)
        {
            xEventGroupSetBitsFromISR(send_state, SEND_COMPLETE_BIT, pdFALSE);
            ESP_LOGD(TAG_MESSAGER, "Send data successfully in tx callback");
        }
        else
        {
            xEventGroupSetBitsFromISR(send_state, SEND_FAIL_BIT, pdFALSE);
            ESP_LOGE(TAG_MESSAGER, "Send data failed in tx callback");
        }
    }

    static void tx_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data)
    {
        // 允许阻塞整个事件循环，事件循环本身就是消息队列

        // 地址判断
        // data的第一个变量是地址位
        if (establish_peer_list)
        {
            // 忙锁，对外可见
            xEventGroupSetBits(send_state, SEND_BUSY);
#ifdef CONFIG_POWERRUNE_TYPE == 1 // 主设备
            uint8_t address = ((uint8_t *)event_data)[0];
            uint8_t *dest_mac = mac_addr[address];
            send_data(++packet_tx_id[address], dest_mac, event_base, &event_id, event_data, ((uint8_t *)event_data)[1]);

#endif
#if ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
            uint8_t *dest_mac = mac_addr;
            send_data(&packet, ++packet_tx_id, dest_mac, event_base, &event_id, event_data, ((uint8_t *)event_data)[1]);

#endif
            // 解除忙锁
            xEventGroupClearBits(send_state, SEND_BUSY);
        }
        else
        {
            ESP_LOGE(TAG_MESSAGER, "Peer list not established, cannot send data");
            return;
        }
    }

    static esp_err_t
    establish_peer_list()
    {

        esp_now_peer_info_t *peer = (esp_now_peer_info_t *)malloc(sizeof(esp_now_peer_info_t));
        if (peer == NULL)
        {
            ESP_LOGE(TAG_MESSAGER, "Malloc broadcast peer information fail");
            return ESP_FAIL;
        }
#ifdef CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
        /* Add broadcast peer information to peer list. */
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = CONFIG_ESPNOW_CHANNEL;
        peer->ifidx = (wifi_interface_t)ESP_IF_WIFI_STA;
        peer->encrypt = false;
        memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
#endif
        // 等待RESPONSE_EVENT或者PING_EVENT
        received_struct *received_data;
        while (1)
        {
            // 从设备只需要向0xFFFFFFFFFFFF发送PING_EVENT广播包即可，直到收到RESPONSE_EVENT包，然后建立peer list
// PING EVENT 数据
#if CONFIG_POWERRUNE_TYPE == 0 // Armour
            int32_t event_id = PRA_PING_EVENT;
            PRA_PING_EVENT_DATA ping_data;
            ping_data.address = ARMOUR1;
            memcpy(&ping_data.config_info, config->get_config_info_pt(), sizeof(PowerRune_Armour_config_info_t));
            send_data(0, broadcast_mac, PRA, &event_id, &ping_data, sizeof(PRA_PING_EVENT_DATA), 0);

            // 发送后延时
            vTaskDelay(100 / portTICK_PERIOD_MS);
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // Motor
            int32_t event_id = PRM_PING_EVENT;
            PRM_PING_EVENT_DATA ping_data;
            ping_data.address = MOTOR;
            memcpy(&ping_data.config_info, config->get_config_info_pt(), sizeof(PowerRune_Motor_config_info_t));
            send_data(0, broadcast_mac, PRM, &event_id, &ping_data, sizeof(PRM_PING_EVENT_DATA), 0);

            // 发送后延时
            vTaskDelay(100 / portTICK_PERIOD_MS);
#endif
            // 接收RESPONSE_EVENT或者PING_EVENT
            if (xQueueReceive(espnow_rx_queue, received_data, portMAX_DELAY) != pdTRUE)
            {
                ESP_LOGE(TAG_MESSAGER, "Failed to receive data from espnow_rx_queue");
                continue;
            }
            uint16_t header = (received_data->data[0] << 8) | received_data->data[1];
            if (header == POWERRUNE_DATA_HEADER)
            {
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
                // 建立peer list，此处peer已经在前述代码中分配内存
                memset(peer, 0, sizeof(esp_now_peer_info_t));
                peer->channel = CONFIG_ESPNOW_CHANNEL;
                peer->ifidx = (wifi_interface_t)ESP_IF_WIFI_STA;
                peer->encrypt = false;
                memcpy(peer->peer_addr, received_data->src_MAC, ESP_NOW_ETH_ALEN);
                ESP_ERROR_CHECK(esp_now_add_peer(peer));
                // 写入mac_addr
                memcpy(mac_addr, received_data->src_MAC, ESP_NOW_ETH_ALEN);
#elif CONFIG_POWERRUNE_TYPE == 1 // RLOGO
                // 解包
                espnow_DATA_pack_t packet = {0};
                packet.header = header;
                packet.pack_id = (received_data->data[2] << 24) |
                                 (received_data->data[3] << 16) |
                                 (received_data->data[4] << 8) | received_data->data[5];
                memcpy(packet.event_base, received_data->data + 6, 4);
                if (packet.event_base != PRA & packet.event_base != PRM)
                {
                    ESP_LOGE(TAG_MESSAGER, "PING packet %i event_base check failed", packet.pack_id);
                    // 释放内存
                    free(received_data->data);
                    continue;
                }
                packet.event_data_len = (received_data->data[10] << 8) | received_data->data[11];
                packet.event_id = received_data->data[12];
                if (packet.event_id != PRA_PING_EVENT & packet.event_id != PRM_PING_EVENT)
                {
                    ESP_LOGE(TAG_MESSAGER, "PING packet %i event_base check failed", packet.pack_id);
                    // 释放内存
                    free(received_data->data);
                    continue;
                }
                memcpy(packet.event_data, received_data->data + 13, packet.event_data_len);
                // CRC校验
                uint16_t crc16 = esp_crc16_le(UINT16_MAX, (uint8_t *)&packet, sizeof(espnow_DATA_pack_t) - 2);
                uint16_t crc16_pack = (received_data->data[6] << 8) | received_data->data[7];
                if (crc16 != crc16_pack)
                {
                    ESP_LOGE(TAG_MESSAGER, "PING packet %i CRC16 check failed", packet.pack_id);
                    // 释放内存
                    free(received_data->data);
                    continue;
                }
                // 建立peer list，此处peer已经在前述代码中分配内存
                memset(peer, 0, sizeof(esp_now_peer_info_t));
                peer->channel = CONFIG_ESPNOW_CHANNEL;
                peer->ifidx = (wifi_interface_t)ESP_IF_WIFI_STA;
                peer->encrypt = false;
                memcpy(peer->peer_addr, received_data->src_MAC, ESP_NOW_ETH_ALEN);
                ESP_ERROR_CHECK(esp_now_add_peer(peer));
                // 识别地址
                if (packet.event_base == PRA)
                {
                    //
                }
                else if (packet.event_base == PRM)
                {
                    memcpy(mac_addr[MOTOR], received_data->src_MAC, ESP_NOW_ETH_ALEN);
                }

#endif

                // 释放内存
                free(received_data->data);
                break;
            }
            else
            {
                // 收到未知包，丢弃
                ESP_LOGE(TAG_MESSAGER, "Unknown packet header: 0x%04X", header);
                // 释放内存
                free(received_data->data);
                continue;
            }
        }
        // 释放内存
        free(peer);
        peer_list_established = true;
        return ESP_OK;
    }

    ESPNowProtocol()
    {
        // 初始化espnow，PMK和LMK
        ESP_ERROR_CHECK(esp_now_init());
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_BELOW));
        // set pmk
        ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

        // 获取自身MAC
        esp_wifi_get_mac((wifi_interface_t)ESP_IF_WIFI_STA, self_mac);

        espnow_rx_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(received_struct));
        assert(espnow_rx_queue != NULL);

        send_state = xEventGroupCreate();

// 注册PING和RESPONSE
#if CONFIG_POWERRUNE_TYPE == 1 // RLOGO
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, RESPONSE_EVENT, tx_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE != 1 // Armour && Motor
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, PING_EVENT, tx_event_handler, NULL));
#endif

        // establish peer list: 发送/接收PING_EVENT广播包，然后接收/回复RESPONSE_EVENT包
        establish_peer_list();
// 发送部分事件注册
#if CONFIG_POWERRUNE_TYPE == 1 // RLogo
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, RESPONSE_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_STOP_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_START_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, tx_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE == 0 // Armour
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, PING_EVENT, tx_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // Motor
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_DISCONNECT_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_SPEED_STABLE_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_START_DONE_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_UNLOCK_DONE_EVENT, tx_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE != 1 // 除了主控外的设备
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, tx_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_COMPLETE_EVENT, tx_event_handler, NULL));
#endif
        esp_now_register_recv_cb(rx_callback);
        esp_now_register_send_cb(tx_callback);
    }
};

// 初始化静态成员
// espnow MAC地址
uint8_t ESPNowProtocol::mac_addr[6][ESP_NOW_ETH_ALEN] = {0};
uint8_t ESPNowProtocol::mac_addr[ESP_NOW_ETH_ALEN] = {0};
// 包ID
#ifdef CONFIG_POWERRUNE_TYPE == 1 // 主设备
uint16_t ESPNowProtocol::packet_tx_id[6] = {0};
uint16_t ESPNowProtocol::packet_rx_id[6] = {0};
#elif ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
uint16_t ESPNowProtocol::packet_tx_id = 0;
uint16_t ESPNowProtocol::packet_rx_id = 0;
#endif
// espnow 数据包队列
QueueHandle_t ESPNowProtocol::espnow_rx_queue = NULL;
// tx mutex
SemaphoreHandle_t ESPNowProtocol::tx_semaphore = xSemaphoreCreateMutex();
// peer list是否成功建立
bool ESPNowProtocol::peer_list_established = false;
// 自身MAC
uint8_t ESPNowProtocol::self_mac[ESP_NOW_ETH_ALEN] = {0};
// 广播地址
uint8_t ESPNowProtocol::broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// 发送数据EventBits
EventGroupHandle_t ESPNowProtocol::send_state = NULL;

#endif