/**
 * @file firmware.h
 * @brief 大符固件支持库，支持Flash Config读写、OTA升级
 * @version 1
 * @date 2024-01-25
 */
// FreeRTOS
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// NVS
#include "nvs_flash.h"
#include "nvs.h"

// Common
#include "esp_event.h"
#include "esp_log.h"

// 大符事件
#include "PowerRune_Events.h"

// Wifi & OTA
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "driver/gpio.h"

#define OTA_BUF_SIZE 512 // OTA缓冲区大小 单位：字节

// LOG TAG
static const char *TAG_FIRMWARE = "Firmware";

// Common Config
struct PowerRune_Common_config_info_t
{
    // 主控MAC地址
    uint8_t main_control_mac[6];
    char URL[256];
    char SSID[20];
    char SSID_pwd[20];
    uint8_t auto_update;
};

// Config
struct PowerRune_Armour_config_info_t
{
    uint8_t brightness;
    uint8_t armour_id; // 若没有设置，则为0xFF
    uint8_t brightness_proportion_matrix;
    uint8_t brightness_proportion_edge;
};
// Config
struct PowerRune_Rlogo_config_info_t
{
    uint8_t brightness;
};
// Config
struct PowerRune_Motor_config_info_t
{
    float kp, ki, kd;
    float i_max, d_max;
    float out_max;
    uint8_t motor_num;
    uint8_t auto_lock;
};

class Config
{
protected:
#if CONFIG_POWER_RUNE_TYPE == 0 // ARMOUR
    static PowerRune_Armour_config_info_t config_info;

#endif
#if CONFIG_POWER_RUNE_TYPE == 1
    static PowerRune_Rlogo_config_info_t config_info;
#endif
#if CONFIG_POWER_RUNE_TYPE == 2 // MOTORCTL
    static PowerRune_Motor_config_info_t config_info;
#endif
    static PowerRune_Common_config_info_t config_common_info;

public:
    static const char *PowerRune_description;
    Config()
    {
        // NVS Flash Init
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init(); // Read config_common_info from NVS

            ESP_ERROR_CHECK(err);

            // Read config_info from NVS
            err = this->read();
        }

        // Read config_info from NVS
        err = this->read();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_FIRMWARE, "read config_info from NVS failed (%s)", esp_err_to_name(err));
            reset();
        }
        // 注册PRC事件处理器
        esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT, (esp_event_handler_t)Config::global_event_handler, this);
    }

// 获取数据指针
#if CONFIG_POWER_RUNE_TYPE == 0 // ARMOUR
    const PowerRune_Armour_config_info_t *get_config_info_pt()
    {
        return &config_info; // Read config_common_info from NVS
    }
#endif
#if CONFIG_POWER_RUNE_TYPE == 1
    const PowerRune_Rlogo_config_info_t *get_config_info_pt()
    { // Read config_common_info from NVS

        return &config_info;
    }
#endif
#if CONFIG_POWER_RUNE_TYPE == 2 // MOTORCTL
    const PowerRune_Motor_config_info_t *get_config_info_pt()
    {
        return &config_info;
    }
#endif
    // Read config_common_info from NVS

    static const PowerRune_Common_config_info_t *get_config_common_info_pt()
    {
        return &config_common_info;
    }

    esp_err_t read()
    {
        // Read config_info from NVS
        nvs_handle_t my_handle;
        ESP_ERROR_CHECK(nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READWRITE, &my_handle));
// BLOB
#if CONFIG_POWER_RUNE_TYPE == 0 // ARMOUR
        size_t required_size = sizeof(PowerRune_Armour_config_info_t);
#endif
#if CONFIG_POWER_RUNE_TYPE == 1
        size_t required_size = sizeof(PowerRune_Rlogo_config_info_t);
#endif
#if CONFIG_POWER_RUNE_TYPE == 2 // MOTORCTL
        size_t required_size = sizeof(PowerRune_Motor_config_info_t);
#endif
        size_t required_size_common = sizeof(PowerRune_Common_config_info_t);
        // Read from Flash
        ESP_ERROR_CHECK(nvs_get_blob(my_handle, "PowerRune Config", &config_info, &required_size));
        ESP_ERROR_CHECK(nvs_get_blob(my_handle, "PowerRune Common Config", &config_common_info, &required_size_common));
        // Close
        nvs_close(my_handle);
        return ESP_OK;
    }

    esp_err_t save()
    {
        // Write config_info to NVS
        nvs_handle_t my_handle;
        ESP_ERROR_CHECK(nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READWRITE, &my_handle));
// BLOB
#if CONFIG_POWER_RUNE_TYPE == 0 // ARMOUR
        size_t required_size = sizeof(PowerRune_Armour_config_info_t);
#endif
#if CONFIG_POWER_RUNE_TYPE == 1
        size_t required_size = sizeof(PowerRune_Rlogo_config_info_t);
#endif
#if CONFIG_POWER_RUNE_TYPE == 2 // MOTORCTL
        size_t required_size = sizeof(PowerRune_Motor_config_info_t);
#endif
        size_t required_size_common = sizeof(PowerRune_Common_config_info_t);
        // Write to Flash
        ESP_ERROR_CHECK(nvs_set_blob(my_handle, "PowerRune Config", &config_info, required_size));
        ESP_ERROR_CHECK(nvs_set_blob(my_handle, "PowerRune Common Config", &config_common_info, required_size_common));
        // Commit written value.
        ESP_ERROR_CHECK(nvs_commit(my_handle));
        // Close
        nvs_close(my_handle);
        return ESP_OK;
    }

    esp_err_t reset()
    {
#if CONFIG_POWER_RUNE_TYPE == 0 // ARMOUR
        config_info = {
            .brightness = 127,
            .armour_id = 0xFF,
            .brightness_proportion_matrix = 127,
            .brightness_proportion_edge = 127,
        };
#endif
#if CONFIG_POWER_RUNE_TYPE == 1
        config_info = {
            .brightness = 127,
        };
#endif
#if CONFIG_POWER_RUNE_TYPE == 2 // MOTORCTL
        // float Kp = 1.2, float Ki = 0.2, float Kd = 0.5, float Pmax = 1000, float Imax = 1000, float Dmax = 1000, float max = 2000
        config_info = {
            .kp = 1.2,
            .ki = 0.2,
            .kd = 0.5,
            .i_max = 1000,
            .d_max = 1000,
            .out_max = 2000,
            .motor_num = 1,
            .auto_lock = 1,
        };
#endif
        config_common_info = {
            .main_control_mac = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
            .URL = CONFIG_DEFAULT_UPDATE_URL, // 需要往后加上CONFIG_DEFAULT_UPDATE_FILE
            .SSID = CONFIG_DEFAULT_UPDATE_SSID,
            .SSID_pwd = CONFIG_DEFAULT_UPDATE_PWD,
            .auto_update = 1,
        };
        this->save();
        return ESP_OK;
    }
    // 事件处理器
    static esp_err_t global_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data)
    {
        esp_err_t err = ESP_OK;

// event_data: PowerRune_Armour/RLogo/Motor_config_info_t, PowerRune_Common_config_info_t, PowerRune_config_info_bitmask_t
#if CONFIG_POWER_RUNE_TYPE == 0 // ARMOUR
        PowerRune_Armour_config_info_t *info = (PowerRune_Armour_config_info_t *)event_data;
#endif
        memcpy(&config_common_info, event_data, sizeof(PowerRune_Common_config_info_t));

        return ESP_OK;
    }
};

#if CONFIG_POWER_RUNE_TYPE == 0 // ARMOUR
const char *Config::PowerRune_description = "Armour";
#endif
#if CONFIG_POWER_RUNE_TYPE == 1
const char *Config::PowerRune_description = "RLogo";
#endif
#if CONFIG_POWER_RUNE_TYPE == 2 // MOTORCTL
const char *Config::PowerRune_description = "Motor";
#endif

// 负责维护硬件启动后的更新，标记更新分区，重启等操作
class Firmware
{
private:
public:
    Firmware()
    {
        // get version from esp-idf esp_description
        esp_app_desc_t app_desc;
        esp_err_t err = esp_ota_get_partition_description(esp_ota_get_running_partition(), &app_desc);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_FIRMWARE, "esp_ota_get_partition_description failed (%s)", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG_FIRMWARE, "PowerRune %s version: %s", Config::PowerRune_description, app_desc.version);
        }
        // 第一次更新重启，Verify更新分区
        if (app_desc.magic_word != ESP_APP_DESC_MAGIC_WORD)
        {
            ESP_LOGW(TAG_FIRMWARE, "First Boot, Verify OTA Partition");
            err = esp_ota_mark_app_valid_cancel_rollback();
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG_FIRMWARE, "esp_ota_mark_app_valid_cancel_rollback failed (%s)", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGI(TAG_FIRMWARE, "OTA Partition Verified");
            }
        }
    }

    static esp_err_t
    global_task_OTA(void *args)
    {
        return firmware->task_OTA();
    }

    esp_err_t event_handler()
    {

        return ESP_OK;
    }

    static err_t global_evenet_handler(void *args)
    {
        Firmware<PowerRune_config_info_t> *firmware = (Firmware<PowerRune_config_info_t> *)args;
        return firmware->event_handler();
    }
    ~Firmware()
    {
        // Implement the function here
    }
};