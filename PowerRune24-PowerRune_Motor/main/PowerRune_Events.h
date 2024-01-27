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

// 公有事件
enum
{
    OTA_BEGIN_EVENT,
    OTA_COMPLETE_EVENT,
    PING_EVENT,
};

// Armour事件
enum
{
    PRA_STOP_EVENT,
    PRA_START_EVENT,
    PRA_HIT_EVENT,
    PRA_COMPLETE_EVENT,
    PRA_CONFIG_EVENT,
    PRA_CONFIG_DONE_EVENT,
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
};

struct PRM_START_DONE_EVENT_DATA
{
    // mode 0 for normal, 1 for trace sin
    int mode;
    // speed in rpm is 10*19
    float amplitude;
    float omega;
    float offset;
};
