#pragma once
#include <firmware.h>
#include "esp_event.h"

// 定义events
// spp服务
ESP_EVENT_DEFINE_BASE(LED_EVENTS);
ESP_EVENT_DEFINE_BASE(URL_EVENTS);
ESP_EVENT_DEFINE_BASE(MAC_EVENTS);
ESP_EVENT_DEFINE_BASE(SSID_EVENTS);
ESP_EVENT_DEFINE_BASE(Wifi_EVENTS);
ESP_EVENT_DEFINE_BASE(AOTA_EVENTS);
ESP_EVENT_DEFINE_BASE(LIT_EVENTS);
ESP_EVENT_DEFINE_BASE(STRIP_LIT_EVENTS);
ESP_EVENT_DEFINE_BASE(R_LIT_EVENTS);
ESP_EVENT_DEFINE_BASE(MATRIX_LIT_EVENTS);
ESP_EVENT_DEFINE_BASE(PID_EVENTS);
ESP_EVENT_DEFINE_BASE(ARMOR_ID_EVENTS);
// ops服务
ESP_EVENT_DEFINE_BASE(RUN_EVENTS);
ESP_EVENT_DEFINE_BASE(GPA_EVENTS);
ESP_EVENT_DEFINE_BASE(UNLK_EVENTS);
ESP_EVENT_DEFINE_BASE(STOP_EVENTS);
ESP_EVENT_DEFINE_BASE(OTA_EVENTS);
// PowerRune_Evets
ESP_EVENT_DEFINE_BASE(PRC);
ESP_EVENT_DEFINE_BASE(PRA);
ESP_EVENT_DEFINE_BASE(PRM);

// 事件循环Handle
esp_event_loop_handle_t pr_events_loop_handle = NULL;

// 公有事件
enum
{
    OTA_BEGIN_EVENT,
    OTA_COMPLETE_EVENT,
    CONFIG_EVENT,
    CONFIG_COMPLETE_EVENT,
    RESPONSE_EVENT,
};
struct CONFIG_OTA_BEGIN_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = 140;
    char url[100];
    char SSID[20];
    char password[20];
}; // Size: 140 + 2
struct CONFIG_OTA_COMPLETE_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = 1;
    esp_err_t status;
}; // Size: 1 + 2
struct CONFIG_ARMOUR_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PowerRune_Armour_config_info_t) + sizeof(PowerRune_Common_config_info_t);
    PowerRune_Armour_config_info_t config_info;
    PowerRune_Common_config_info_t config_common_info;
};
struct CONFIG_MOTOR_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PowerRune_Motor_config_info_t) + sizeof(PowerRune_Common_config_info_t);
    PowerRune_Motor_config_info_t config_info;
    PowerRune_Common_config_info_t config_common_info;
};
struct CONFIG_RLOGO_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PowerRune_Rlogo_config_info_t) + sizeof(PowerRune_Common_config_info_t);
    PowerRune_Rlogo_config_info_t config_info;
    PowerRune_Common_config_info_t config_common_info;
};
struct RESPONSE_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len;
};

// Armour事件
enum
{
    PRA_STOP_EVENT,
    PRA_START_EVENT,
    PRA_HIT_EVENT,
    PRA_COMPLETE_EVENT,
    PRA_PING_EVENT,
};

struct PRA_PING_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PowerRune_Armour_config_info_t);
    PowerRune_Armour_config_info_t config_info;
};

// 电机事件
enum
{
    PRM_UNLOCK_EVENT,
    PRM_UNLOCK_DONE_EVENT,
    PRM_START_EVENT,
    PRM_START_DONE_EVENT,
    PRM_SPEED_STABLE_EVENT,
    PRM_STOP_EVENT,
    PRM_DISCONNECT_EVENT,
    PRM_PING_EVENT,
};

struct PRM_PING_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PowerRune_Motor_config_info_t);
    PowerRune_Motor_config_info_t config_info;
};