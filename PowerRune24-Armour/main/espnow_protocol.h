/**
 * @file espnow_protocol.h
 * @brief ESP-NOW协议
 * @version 0.1
 * @date 2024-02-18
 * @note 用于ESP-NOW通信
*/
#pragma once
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
#include "firmware.h"
#include "LED.h"
#include "PowerRune_Events.h"

// espnow 数据包队列大小
#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
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

#pragma pack(1)
// espnow 数据包，取消字节对齐
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
} espnow_DATA_pack_t;                   // Size: 2 + 2 + 2 + 4 + 2 + 1 + 3+196 + 1 + 6 + 1 = 220

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

typedef struct
{
    EventGroupHandle_t event_group;
    uint8_t mac_addr_new[ESP_NOW_ETH_ALEN];
} reset_armour_id_t;

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
    static uint8_t NULL_mac[ESP_NOW_ETH_ALEN];
// espnow MAC地址, 使用静态多态，对主控(收发6个设备, 5个ESP32S3[0:4], 1个ESP32C3[5])和从控(6个分设备发送)的MAC地址
#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
    static uint8_t mac_addr[6][ESP_NOW_ETH_ALEN];
    // 包ID，一方的TX_ID随包发送，原则上应该比对方的RX_ID大1，否则说明有包丢失
    static uint16_t packet_tx_id[6];
    static uint16_t packet_rx_id[6];
    static uint8_t mac_to_address(uint8_t *mac);
    static void reset_id_PRA_HIT_handler(void *event_handler_arg,
                                         esp_event_base_t event_base,
                                         int32_t event_id,
                                         void *event_data);
    esp_err_t reset_armour_id();
#elif ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
    static uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    // 包ID
    static uint16_t packet_tx_id;
    static uint16_t packet_rx_id;
    // becon time
    static TickType_t beacon_send_time;
    static TickType_t beacon_time;
    static uint8_t beacon_mode;
    static TaskHandle_t beacon_task_handle;
    static esp_event_handler_t beacon_timeout_handler;
#endif
    // #endif
    static QueueHandle_t espnow_rx_queue;
    // tx mutex
    static SemaphoreHandle_t tx_semaphore;
    // reset_id struct

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
    static void log_packet(const espnow_DATA_pack_t *data, uint16_t crc16 = 0);
    static void log_raw_packet(const uint8_t *data, uint8_t len);
    static void log_packet(const ACK_OK_pack_t *data);
    static void log_packet(const ACK_FAIL_pack_t *data);
    static const int SEND_COMPLETE_BIT = BIT0;    // 已发送，正在等待ACK，内部使用
    static const int SEND_FAIL_BIT = BIT1;        // 发送失败（未传出），内部使用
    static const int SEND_ACK_PENDING_BIT = BIT2; // 已发送，等待ACK，内部使用

public:
    // 自身MAC
    static uint8_t self_mac[ESP_NOW_ETH_ALEN];
    // 广播地址
    static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN];
    // 发送数据EventBits
    static EventGroupHandle_t send_state;
    // 先置位BUSY，然后发送数据，发送完成置位COMPLETE，等待ACK，收到ACK后置位OK，超时重传，清除BUSY和COMPLETE
    static const int SEND_ACK_OK_BIT = BIT3;      // 已发送，收到ACK_OK包，外部可读
    static const int SEND_ACK_FAIL_BIT = BIT4;    // 已发送，收到ACK_FAIL包，外部可读
    static const int SEND_BUSY = BIT5;            // 正在发送，外部可读，内部可用于判断是否接受ACK包
    static const int SEND_ACK_TIMEOUT_BIT = BIT6; // 已发送，等待ACK超时

    static void rx_callback(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len);

/**
 * @brief 维护Beacon
 */
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
    static void beacon_task(void *pvParameter);
#endif
    /**
     * @brief 解析espnow数据包
     * @param pvParameter 任务参数
     */
    static void parse_data_task(void *pvParameter);

    // data_len为事件数据长度
    static esp_err_t send_data(uint16_t packet_tx_id, uint8_t *dest_mac, esp_event_base_t event_base, int32_t event_id, void *data, uint16_t data_len, uint8_t wait_ack = 1, uint8_t mutex = 1);

    static esp_err_t send_ACK(uint16_t packet_tx_id, uint8_t *dest_mac, PacketType type);

    static void tx_callback(const uint8_t *mac_addr, esp_now_send_status_t status);

    static void tx_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data);
    

    // 接受传入NULL，表示第一次建立peer list，否则表示接收到中途复位的设备，需要发送RESPONSE_EVENT
    static esp_err_t
    establish_peer_list(uint8_t *response_mac = NULL);

    static esp_err_t beacon_control(uint8_t mode);

    ESPNowProtocol(esp_event_handler_t);
};