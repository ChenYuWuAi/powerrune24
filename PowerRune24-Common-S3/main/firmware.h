/**
 * @file firmware.h
 * @brief 大符固件库，支持Flash Config读写、OTA升级
 * @version 1
 * @date 2024-01-25
 */
// includes
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

// NVS
#include "nvs_flash.h"
#include "nvs.h"

// OTA

#include "driver/gpio.h"

// LOG TAG
static const char *TAG_FIRMWARE = "Firmware";

// Common Config
struct PowerRune_Common_config_info_t
{
    char main_control_mac[6]; // 主控制器MAC地址
    char URL[256];
    char SSID[20];
    char SSID_pwd[20];
    uint8_t auto_update;
};

template <typename PowerRune_config_info_t>
class Config
{
protected:
    PowerRune_config_info_t config_info;
    PowerRune_Common_config_info_t config_common_info;

public:
    Config()
    {
        // NVS Flash Init
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
            ESP_ERROR_CHECK(err);

            reset();
        }
    }

    const PowerRune_config_info_t *get_config_info_pt()
    {
        return &config_info;
    }

    const PowerRune_config_info_t *get_config_common_info_pt()
    {
        return &config_common_info;
    }

    esp_err_t read()
    {
        // Read config_info from NVS
        nvs_handle_t my_handle;
        ESP_ERROR_CHECK(nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &my_handle));
        // BLOB
        size_t required_size = sizeof(PowerRune_config_info_t);
        size_t required_size_common = sizeof(PowerRune_Common_config_info_t);
        // Read from Flash
        ESP_ERROR_CHECK(nvs_get_blob(my_handle, "PowerRune Config", &config_info, &required_size));
        ESP_ERROR_CHECK(nvs_get_blob(my_handle, "PowerRune Common Config", &config_common_info, &required_size_common));
        // Close
        ESP_ERROR_CHECK(nvs_close(my_handle));
        return ESP_OK;
    }

    esp_err_t save()
    {
        // Write config_info to NVS
        nvs_handle_t my_handle;
        ESP_ERROR_CHECK(nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &my_handle));
        // BLOB
        size_t required_size = sizeof(PowerRune_config_info_t);
        size_t required_size_common = sizeof(PowerRune_Common_config_info_t);
        // Write to Flash
        ESP_ERROR_CHECK(nvs_set_blob(my_handle, "PowerRune Config", &config_info, required_size));
        ESP_ERROR_CHECK(nvs_set_blob(my_handle, "PowerRune Common Config", &config_common_info, required_size_common));
        // Commit written value.
        ESP_ERROR_CHECK(nvs_commit(my_handle));
        // Close
        ESP_ERROR_CHECK(nvs_close(my_handle));
        return ESP_OK;
    }

    esp_err_t reset()
    {
        config_common_info = {
            .main_control_mac = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
            .URL = CONFIG_DEFAULT_UPDATE_URL, // 需要往后加上CONFIG_DEFAULT_UPDATE_FILE
            .SSID = CONFIG_DEFAULT_UPDATE_SSID,
            .SSID_pwd = CONFIG_DEFAULT_UPDATE_PWD,
            .auto_update = 1,
        };
        this->save(config_common_info);
        return ESP_OK;
    }
};

template <typename PowerRune_config_info_t>
class Firmware : public Config<PowerRune_config_info_t>
{
private:

    enum PR_type
    {
        ARMOUR,
        RLOGO,
        MOTORCTL,
    };

    struct board_info_t
    {
        PR_type board_type;
        uint8_t board_id; // 仅对于ARMOUR有效 Range[1:5]
        char version[25]; // 程序版本号，从esp_app_desc_t中获取
        Config<PowerRune_config_info_t> config;
    } board_info;

public:
    Firmware()
    {
    }

    esp_err_t task_OTA()
    {
        // Implement the function here
        return ESP_OK;
    }

    static esp_err_t global_task_OTA(void *args)
    {
        Firmware<PowerRune_config_info_t> *firmware = (Firmware<PowerRune_config_info_t> *)args;
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