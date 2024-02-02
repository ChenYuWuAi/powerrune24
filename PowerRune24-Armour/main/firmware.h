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
#include "freertos/event_groups.h"

// NVS
#include "nvs_flash.h"
#include "nvs.h"

// Common
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"

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

// Config class variable
extern Config *config = NULL;

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

    static esp_err_t read()
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

    static esp_err_t save()
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

    static esp_err_t reset()
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
        Config::save();
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
        memcpy(&config_info, event_data, sizeof(PowerRune_Armour_config_info_t));
        // Write config_info to NVS
        Config::save();
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
    esp_app_desc_t app_desc;
    uint8_t sha_256[32] = {0};
    // OTA Task handler
    TaskHandle_t *ota_task_handle;

public:
    Firmware()
    {
        if (config == NULL)
            config = new Config();
        const esp_partition_t *running = esp_ota_get_running_partition();
        // get version from esp-idf esp_description
        esp_err_t err = esp_ota_get_partition_description(running, &app_desc);
        // get sha256 digest for running partition
        esp_partition_get_sha256(running, sha_256);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_FIRMWARE, "esp_ota_get_partition_description failed (%s)", esp_err_to_name(err));
        }
        else
        {
            // print sha256
            char hash_print[65];
            hash_print[64] = 0;
            for (int i = 0; i < 32; ++i)
            {
                sprintf(&hash_print[64], "%02x", sha_256[i]);
            }
            ESP_LOGI(TAG_FIRMWARE, "PowerRune %s version: %s, Hash: %s", Config::PowerRune_description, app_desc.version, hash_print);
        }
        // 完成OTA初始化
        // 启动Wifi
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_GOTO_ON_ERROR(esp_event_loop_create_default(), err_out);
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_GOTO_ON_ERROR(esp_wifi_init(&cfg), err_out);

        esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
        // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
        esp_netif_config.if_desc = "STA";
        esp_netif_config.route_prio = 128;
        esp_netif_t *netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);

        esp_wifi_set_default_wifi_sta_handlers();

        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        esp_wifi_set_ps(WIFI_PS_NONE);
        if (config->get_config_common_info_pt()->auto_update)
        {
            // Start OTA
            xTaskCreate((TaskFunction_t)Firmware::task_OTA, "OTA", 4096, NULL, 5, ota_task_handle);
        }
    }

    static esp_err_t task_OTA(void *args)
    {
        esp_err_t err = ESP_OK;

        // Get Config
        const PowerRune_Common_config_info_t *config_common_info = Config::get_config_common_info_pt();
        // Connect Wifi
        wifi_config_t wifi_config = {
            .sta = {
                .ssid = config->get_config_common_info_pt()->SSID,
                .password = config->get_config_common_info_pt()->SSID_pwd,
                .scan_method = WIFI_FAST_SCAN,
                .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
                .threshold = {
                    .rssi = -127,
                    .authmode = strlen(config->get_config_common_info_pt()->SSID_pwd) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
                },
            },
        };
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

        // Start Wifi Connection & EventGroup Bits
        ESP_LOGI(TAG_FIRMWARE, "Connecting to %s...", config->get_config_common_info_pt()->SSID);
        // Set Semaphore
        EventGroupHandle_t wifi_event_group = xEventGroupCreate();
        // 注册连接事件监听
        esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, (esp_event_handler_t)Firmware::global_system_event_handler, &wifi_event_group);
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, (esp_event_handler_t)Firmware::global_system_event_handler, &wifi_event_group);

        ESP_ERROR_CHECK(esp_wifi_connect());
        // Wait for connection
        ESP_LOGI(TAG_FIRMWARE, "Waiting for connection");
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
        ESP_LOGI(TAG_FIRMWARE, "Connected to AP");

        ESP_LOGI(TAG_FIRMWARE, "OTA task complete");
        return ESP_OK;
    err_out:
        ESP_LOGE(TAG_FIRMWARE, "OTA task failed (%s)", esp_err_to_name(err));
        // OTA_COMPLETE_EVENT with err
        esp_event_post_to(pr_events_loop_handle, OTA_EVENTS, OTA_COMPLETE_EVENT, &err, sizeof(esp_err_t), portMAX_DELAY);
        return err;
    }

    static esp_err_t global_system_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data)
    {
        if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        {
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
        else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
        {
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }

        return ESP_OK;
    }

    // 处理系统事件，例如等待Wifi连接，获取IP地址等
    static static err_t global_event_ ~Firmware()
    {
        // Implement the function here
    }
};