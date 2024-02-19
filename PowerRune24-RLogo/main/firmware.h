/**
 * @file firmware.h
 * @brief 大符固件支持库，支持Flash Config读写、OTA升级
 * @version 1.0
 * @date 2024-02-03
 */
// FreeRTOS
#pragma once
#ifndef _FIRMWARE_H_
#define _FIRMWARE_H_
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// NVS
#include "nvs_flash.h"
#include "nvs.h"

// Common
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

// Wifi & OTA
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "driver/gpio.h"

// LED
#include "LED.h"

// OTA缓冲区大小 单位：字节
#define OTA_BUF_SIZE 1024
// OTA时清除flash
#define ERASE_NVS_FLASH_WHEN_OTA

// Common Config
struct PowerRune_Common_config_info_t
{
    char URL[100];
    char SSID[20];
    char SSID_pwd[20];
    uint8_t auto_update;
};

// Config
struct PowerRune_Armour_config_info_t
{
    uint8_t brightness;
    uint8_t armour_id; // 应该从1开始，若没有设置，则为0x00
    uint8_t brightness_proportion_matrix;
    uint8_t brightness_proportion_edge;
};

struct PowerRune_Rlogo_config_info_t
{
    uint8_t brightness;
};

struct PowerRune_Motor_config_info_t
{
    float kp, ki, kd;
    float i_max, d_max;
    float out_max;
    uint8_t motor_num;
    uint8_t auto_lock;
};

// 大符事件依赖于上方前向声明
#include "PowerRune_Events.h"

class Config
{
protected:
#if CONFIG_POWERRUNE_TYPE == 0 // ARMOUR
    static PowerRune_Armour_config_info_t config_info;

#endif
#if CONFIG_POWERRUNE_TYPE == 1
    static PowerRune_Rlogo_config_info_t config_info;
    static PowerRune_Armour_config_info_t config_armour_info[5];
    static PowerRune_Motor_config_info_t config_motor_info;
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // MOTORCTL
    static PowerRune_Motor_config_info_t config_info;
#endif
    static PowerRune_Common_config_info_t config_common_info;

public:
    static const char *PowerRune_description;

    Config();

// 获取数据指针
#if CONFIG_POWERRUNE_TYPE == 0 // ARMOUR
    const PowerRune_Armour_config_info_t *get_config_info_pt();
#endif
#if CONFIG_POWERRUNE_TYPE == 1
    const PowerRune_Rlogo_config_info_t *get_config_info_pt();

    const PowerRune_Armour_config_info_t *get_config_armour_info_pt(uint8_t index);

    const PowerRune_Motor_config_info_t *get_config_motor_info_pt();
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // MOTORCTL
    const PowerRune_Motor_config_info_t *get_config_info_pt();
#endif
    // Read config_common_info from NVS

    static const PowerRune_Common_config_info_t *get_config_common_info_pt();

    static esp_err_t read();

    static esp_err_t save();

    static esp_err_t reset();
    // 事件处理器
    static void global_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data);
};

extern Config *config;

// 负责维护硬件启动后的更新，标记更新分区，重启等操作
class Firmware
{
private:
    static esp_app_desc_t app_desc;
    uint8_t sha_256[32] = {0};
    // WIFI EventBits
    static const int WIFI_CONNECTED_BIT = BIT0;
    static const int WIFI_FAIL_BIT = BIT1;
    // firmware info
    const esp_partition_t *running;

    // netif handle
    static esp_netif_t *netif;

    /**
     * @brief HTTP协议栈Wifi初始化
     *
     */
    static esp_err_t wifi_ota_init();

    static esp_err_t wifi_connect(const PowerRune_Common_config_info_t *config_common_info, uint8_t retryNum);

    static void wifi_disconnect();

    static void http_cleanup(esp_http_client_handle_t client);

public:
    static const int OTA_COMPLETE_BIT = BIT2;
    static const int OTA_COMPLETE_LISTENING_BIT = BIT3;
    static EventGroupHandle_t ota_event_group;
    static QueueHandle_t ota_complete_queue;
    Firmware();

    // args: 0: 不重启 1: 重启 2: 重设ESP_NOW通道，回复更新状态后，重启
    static void task_OTA(void *args);

    static void global_system_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void global_pr_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
};
#endif