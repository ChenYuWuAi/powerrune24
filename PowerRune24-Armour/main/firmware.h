/**
 * @file firmware.h
 * @brief 大符固件支持库，支持Flash Config读写、OTA升级
 * @version 1.0
 * @date 2024-02-03
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

// Wifi & OTA
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "driver/gpio.h"

// OTA缓冲区大小 单位：字节
#define OTA_BUF_SIZE 1024
#define ERASE_NVS_FLASH_WHEN_OTA

// LOG TAG
static const char *TAG_FIRMWARE = "Firmware";

// 事件循环Handle
extern esp_event_loop_handle_t pr_events_loop_handle;

// Common Config
struct PowerRune_Common_config_info_t
{
    // 主控MAC地址
    uint8_t main_control_mac[6];
    char URL[200];
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
        ESP_LOGI(TAG_FIRMWARE, "Loading Config");
        // NVS Flash Init
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_LOGI(TAG_FIRMWARE, "NVS Flash not established. Establishing...");
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init(); // Read config_common_info from NVS
            if (err == ESP_OK)
                this->reset();
            ESP_LOGI(TAG_FIRMWARE, "PowerRune config established.");
        }
        else
        {
            // Read config_info from NVS
            err = this->read();
            if (err == ESP_ERR_NVS_NOT_FOUND)
                err = this->reset();
        }
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_FIRMWARE, "read config_info from NVS failed (%s)", esp_err_to_name(err));
            reset();
        }
        // 注册PRC事件处理器
        esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT, (esp_event_handler_t)Config::global_event_handler, this);
        ESP_LOGI(TAG_FIRMWARE, "Configuration Complete");
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
        esp_err_t ret = ESP_OK;
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
        ESP_GOTO_ON_ERROR(nvs_get_blob(my_handle, "pr_cfg", &config_info, &required_size), error, TAG_FIRMWARE, "Failed to read from NVS flash");
        ESP_GOTO_ON_ERROR(nvs_get_blob(my_handle, "pr_cmn_cfg", &config_common_info, &required_size_common), error, TAG_FIRMWARE, "Failed to read from NVS flash");
        // Close
        nvs_close(my_handle);
        return ESP_OK;
    error:
        return ret;
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
        ESP_ERROR_CHECK(nvs_set_blob(my_handle, "pr_cfg", &config_info, required_size));                   // 标准配置结构体的密钥
        ESP_ERROR_CHECK(nvs_set_blob(my_handle, "pr_cmn_cfg", &config_common_info, required_size_common)); // 公有配置结构体的密钥
        // Commit written value.
        ESP_ERROR_CHECK(nvs_commit(my_handle));
        // Close
        nvs_close(my_handle);
        ESP_LOGI(TAG_FIRMWARE, "Configuration Saved.");
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
    static void global_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data)
    {
        esp_err_t err = ESP_OK;
        if (id == CONFIG_EVENT)
        {
            memcpy(&config_common_info, event_data, sizeof(PowerRune_Common_config_info_t));
// event_data: PowerRune_Common_config_info_t, PowerRune_Armour/RLogo/Motor_config_info_t
#if CONFIG_POWER_RUNE_TYPE == 0 // ARMOUR
            PowerRune_Armour_config_info_t *info = (PowerRune_Armour_config_info_t *)event_data + sizeof(PowerRune_Common_config_info_t);
            memcpy(&config_info, info, sizeof(PowerRune_Armour_config_info_t));
#endif
#if CONFIG_POWER_RUNE_TYPE == 1
            PowerRune_Rlogo_config_info_t *info = (PowerRune_Rlogo_config_info_t *)event_data + sizeof(PowerRune_Common_config_info_t);
            memcpy(&config_info, info, sizeof(PowerRune_Rlogo_config_info_t));
#endif
#if CONFIG_POWER_RUNE_TYPE == 2 // MOTORCTL
            PowerRune_Motor_config_info_t *info = (PowerRune_Motor_config_info_t *)event_data + sizeof(PowerRune_Common_config_info_t);
            memcpy(&config_info, info, sizeof(PowerRune_Motor_config_info_t));
#endif
            // Write config_info to NVS
            Config::save();
            // Post Complete Event
            esp_event_post_to(pr_events_loop_handle, PRC, CONFIG_COMPLETE_EVENT, &err, sizeof(esp_err_t), portMAX_DELAY);
        }
    }
};

// Config class variable
Config *config = NULL;

#if CONFIG_POWER_RUNE_TYPE == 0 // ARMOUR
const char *Config::PowerRune_description = "Armour";
PowerRune_Armour_config_info_t Config::config_info = {0};
#endif
#if CONFIG_POWER_RUNE_TYPE == 1
const char *Config::PowerRune_description = "RLogo";
PowerRune_Rlogo_config_info_t Config::config_info = {0};
#endif
#if CONFIG_POWER_RUNE_TYPE == 2 // MOTORCTL
const char *Config::PowerRune_description = "Motor";
PowerRune_Motor_config_info_t Config::config_info = {0};
#endif
PowerRune_Common_config_info_t Config::config_common_info = {0};

// 负责维护硬件启动后的更新，标记更新分区，重启等操作
class Firmware
{
private:
    static esp_app_desc_t app_desc;
    uint8_t sha_256[32] = {0};
    // OTA Task handler
    TaskHandle_t ota_task_handle;
    // WIFI EventBits
    static const int WIFI_CONNECTED_BIT = BIT0;
    static const int WIFI_FAIL_BIT = BIT1;
    static const int OTA_COMPLETE_BIT = BIT2;
    // firmware info
    const esp_partition_t *running;

    // netif handle
    static esp_netif_t *netif;
    static esp_err_t wifi_init()
    {
        // 启动Wifi
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
        // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
        esp_netif_config.route_prio = 128;
        netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
        esp_wifi_set_default_wifi_sta_handlers();

        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());

        esp_wifi_set_ps(WIFI_PS_NONE);
        return ESP_OK;
    }
    static esp_err_t wifi_connect(const PowerRune_Common_config_info_t *config_common_info, uint8_t retryNum)
    {
        // Retry counter
        uint8_t retry = 0;
        // Connect Wifi
        wifi_config_t wifi_config = {
            .sta = {
                .scan_method = WIFI_FAST_SCAN,
                .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
                .threshold = {
                    .rssi = -127,
                    .authmode = strlen(config->get_config_common_info_pt()->SSID_pwd) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
                },
            },
        };
        strcpy((char *)wifi_config.sta.ssid, config_common_info->SSID);
        strcpy((char *)wifi_config.sta.password, config_common_info->SSID_pwd);

        // Start Wifi Connection & EventGroup Bits
        ESP_LOGI(TAG_FIRMWARE, "Connecting to %s...", config->get_config_common_info_pt()->SSID);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

        // Set Semaphore
        EventGroupHandle_t wifi_event_group = xEventGroupCreate();
        // 注册连接事件监听
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, (esp_event_handler_t)Firmware::global_system_event_handler, &wifi_event_group);
        esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, (esp_event_handler_t)Firmware::global_system_event_handler, &wifi_event_group);

        // Establish connection
        EventBits_t bits;
        do
        {
            // Connect to AP
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK && err != ESP_ERR_WIFI_CONN)
                ESP_ERROR_CHECK(err);
            // Wait for connection
            ESP_LOGI(TAG_FIRMWARE, "Waiting for connection...%i", retry);
            bits = xEventGroupWaitBits(wifi_event_group, Firmware::WIFI_CONNECTED_BIT | Firmware::WIFI_FAIL_BIT, true, false, 20000 / portTICK_PERIOD_MS);
            if (retry++ == retryNum && bits != Firmware::WIFI_CONNECTED_BIT)
            {
                ESP_LOGE(TAG_FIRMWARE, "Connect to AP failed");
                return ESP_ERR_TIMEOUT;
            }
        } while (bits != Firmware::WIFI_CONNECTED_BIT);
        // 释放事件组
        vEventGroupDelete(wifi_event_group);
        // 注销事件监听
        esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, Firmware::global_system_event_handler);
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, Firmware::global_system_event_handler);
        ESP_LOGI(TAG_FIRMWARE, "Connected to AP");
        return ESP_OK;
    }

    static void wifi_disconnect()
    {
        // 断开连接
        ESP_ERROR_CHECK(esp_wifi_disconnect());
    }

    static void http_cleanup(esp_http_client_handle_t client)
    {
        if (client == NULL)
            return;
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
    }

public:
    Firmware()
    {
        // 常亮LED
        led->set_mode(0, 1);

        running = esp_ota_get_running_partition(); // 验证固件
        esp_ota_img_states_t ota_state;
        if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
        {
            if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
            {
#ifdef ERASE_NVS_FLASH_WHEN_OTA
                esp_err_t err = ESP_OK;
                ESP_LOGI(TAG_FIRMWARE, "Erasing NVS flash...");
                ESP_ERROR_CHECK(nvs_flash_erase());
                err = nvs_flash_init(); // Read config_common_info from NVS
                if (err == ESP_OK)
                    Config::reset();
#endif
                ESP_LOGI(TAG_FIRMWARE, "Diagnostics completed successfully! Continuing execution ...");
                esp_ota_mark_app_valid_cancel_rollback();
            }
        }
        if (config == NULL)
            config = new Config();
        // get version from esp-idf esp_description
        esp_err_t err = esp_ota_get_partition_description(running, &app_desc);
        // get sha256 digest for running partition
        esp_partition_get_sha256(running, sha_256);

        if (err != ESP_OK)
            ESP_LOGE(TAG_FIRMWARE, "esp_ota_get_partition_description failed (%s)", esp_err_to_name(err));
        else
        {
            // print sha256
            char hash_print[65];
            hash_print[64] = 0;
            for (int i = 0; i < 32; ++i)
            {
                sprintf(&hash_print[i * 2], "%02x", sha_256[i]);
            }
            ESP_LOGI(TAG_FIRMWARE, "PowerRune %s version: %s, Hash: %s", Config::PowerRune_description, app_desc.version, hash_print);
        }
        // 启动Wifi
        wifi_init();
        if (config->get_config_common_info_pt()->auto_update)
        {
            // Start OTA
            ESP_LOGI(TAG_FIRMWARE, "Auto update enabled. Starting OTA task...");

            // led闪烁
            led->set_mode(2, 0);
            xTaskCreate((TaskFunction_t)Firmware::task_OTA, "OTA", 8192, NULL, 5, &ota_task_handle);
            // Wait for OTA_COMPLETE_EVENT
            EventGroupHandle_t ota_event_group = xEventGroupCreate();
            esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, (esp_event_handler_t)Firmware::global_pr_event_handler, ota_event_group);
            xEventGroupWaitBits(ota_event_group, OTA_COMPLETE_BIT, false, true, portMAX_DELAY);
            // led停止
            led->set_mode(1, 0);
        }
        ESP_LOGI(TAG_FIRMWARE, "Firmware init complete");
    }

    static void task_OTA(void *args)
    {
        // Variables
        esp_err_t err = ESP_OK;
        esp_ota_handle_t update_handle = 0;
        const esp_partition_t *update_partition = NULL;
        int read_length = 0;
        esp_http_client_handle_t client = NULL;
        int content_length;
        int count = 0;
        bool image_header_was_checked = false;

        // Buffer
        char ota_write_data[OTA_BUF_SIZE + 1] = {0};

        // Get Config
        const PowerRune_Common_config_info_t *config_common_info = Config::get_config_common_info_pt();
        // URL 处理
        char file_url[200] = {0};
        snprintf(file_url, 200, CONFIG_DEFAULT_UPDATE_FILE, config_common_info->URL, CONFIG_POWERRUNE_TYPE);
        ESP_LOGI(TAG_FIRMWARE, "Update URL: %s", file_url);
        esp_http_client_config_t config = {
            .url = file_url,
            .cert_pem = NULL,
            .timeout_ms = CONFIG_OTA_TIMEOUT,
            .keep_alive_enable = true,
        };
        // Get running partition
        const esp_partition_t *configured = esp_ota_get_boot_partition();
        const esp_partition_t *running = esp_ota_get_running_partition();

        // Check if running partition is the same as configured
        if (configured != running)
        {
            ESP_LOGW(TAG_FIRMWARE, "Configured OTA boot partition at offset 0x%08" PRIx32 ", but running from offset 0x%08" PRIx32,
                     configured->address, running->address);
            ESP_LOGW(TAG_FIRMWARE, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
        }
        ESP_LOGI(TAG_FIRMWARE, "Running partition type %d subtype %d (offset 0x%08" PRIx32 ")",
                 running->type, running->subtype, running->address);

        // Connect Wifi
        err = wifi_connect(config_common_info, 5); // Retry 3 times
        if (err != ESP_OK)
        {
            goto ret;
        }
        // HTTP Client
        ESP_LOGI(TAG_FIRMWARE, "Starting HTTP Client");

        client = esp_http_client_init(&config);
        if (client == NULL)
        {
            ESP_LOGE(TAG_FIRMWARE, "Failed to initialise HTTP connection");
            goto ret;
        }
        err = esp_http_client_open(client, 0);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_FIRMWARE, "Failed to open HTTP connection: %s", esp_err_to_name(err));
            goto ret;
        }
        content_length = esp_http_client_fetch_headers(client);
        if (content_length <= 0)
        {
            ESP_LOGE(TAG_FIRMWARE, "OTA file size is 0");
            err = ESP_ERR_NOT_FOUND;
            goto ret;
        }
        else
        {
            ESP_LOGI(TAG_FIRMWARE, "OTA file size is %d", content_length);
        }
        update_partition = esp_ota_get_next_update_partition(NULL);
        assert(update_partition != NULL);
        // HTTP stream receive
        while (1)
        {
            int data_read = esp_http_client_read(client, ota_write_data, OTA_BUF_SIZE);
            if (data_read < 0)
            {
                ESP_LOGE(TAG_FIRMWARE, "Error: SSL data read error");
                goto ret;
            }
            else if (data_read > 0)
            {
                if (image_header_was_checked == false)
                {
                    esp_app_desc_t new_app_info;
                    if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                    {
                        // check current version with downloading
                        memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                        ESP_LOGI(TAG_FIRMWARE, "New firmware version: %s", new_app_info.version);

                        const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                        esp_app_desc_t invalid_app_info;
                        if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                        {
                            ESP_LOGI(TAG_FIRMWARE, "Last invalid firmware version: %s", invalid_app_info.version);
                        }
                        // version check
                        if (last_invalid_app != NULL)
                        {
                            if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                            {
                                err = ESP_ERR_NOT_SUPPORTED;
                                ESP_LOGW(TAG_FIRMWARE, "New version is the same as invalid version.");
                                ESP_LOGW(TAG_FIRMWARE, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                                ESP_LOGW(TAG_FIRMWARE, "The firmware has been rolled back to the previous version.");
                                goto ret;
                            }
                        }
                        if (memcmp(new_app_info.version, Firmware::app_desc.version, sizeof(new_app_info.version)) == 0)
                        {
                            ESP_LOGW(TAG_FIRMWARE, "Current running version is the same as a new. We will not continue the update.");
                            err = ESP_ERR_NOT_SUPPORTED;
                            goto ret;
                        }
                        image_header_was_checked = true;
                        ESP_LOGI(TAG_FIRMWARE, "Writing firmware to partition subtype %d at offset 0x%" PRIx32,
                                 update_partition->subtype, update_partition->address);
                        err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG_FIRMWARE, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                            goto ret;
                        }
                        ESP_LOGI(TAG_FIRMWARE, "Downloading and flashing firmware...");
                    }
                    else
                    {
                        ESP_LOGE(TAG_FIRMWARE, "received package is not fit len");
                        err = ESP_ERR_OTA_VALIDATE_FAILED;
                        goto ret;
                    }
                }

                err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG_FIRMWARE, "esp_ota_write failed (%s)", esp_err_to_name(err));
                    esp_ota_abort(update_handle);
                    goto ret;
                }
                read_length += data_read;
                count++;
                if (count % 5 == 0)
                    ESP_LOGI(TAG_FIRMWARE, "Written image length %d, progress %i %%.", read_length,
                             (int)((float)read_length / (float)content_length * 100));
            }
            else if (data_read == 0)
            {
                /*
                 * As esp_http_client_read never returns negative error code, we rely on
                 * `errno` to check for underlying transport connectivity closure if any
                 */
                if (errno == ECONNRESET || errno == ENOTCONN)
                {
                    ESP_LOGE(TAG_FIRMWARE, "Connection closed, errno = %d", errno);
                    break;
                }
                if (esp_http_client_is_complete_data_received(client) == true)
                {
                    ESP_LOGI(TAG_FIRMWARE, "Connection closed");
                    break;
                }
            }
        }
        // Length
        ESP_LOGI(TAG_FIRMWARE, "Total Write binary data length: %d", content_length);
        if (esp_http_client_is_complete_data_received(client) != true)
        {
            ESP_LOGE(TAG_FIRMWARE, "Error in receiving complete file");
            err = ESP_ERR_OTA_VALIDATE_FAILED;
            goto ret;
        }
        // Verify firmware
        err = esp_ota_end(update_handle);
        if (err != ESP_OK)
        {
            if (err == ESP_ERR_OTA_VALIDATE_FAILED)
            {
                ESP_LOGE(TAG_FIRMWARE, "Image validation failed, image is corrupted");
            }
            else
            {
                ESP_LOGE(TAG_FIRMWARE, "esp_ota_end failed (%s)!", esp_err_to_name(err));
            }
            goto ret;
        }
        err = esp_ota_set_boot_partition(update_partition);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_FIRMWARE, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
            goto ret;
        }
        // post OTA_COMPLETE_EVENT and wait for ack
        ESP_LOGI(TAG_FIRMWARE, "OTA complete, ready to restart");
        err = ESP_OK;
        esp_event_post_to(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, &err, sizeof(esp_err_t), portMAX_DELAY);
        vTaskDelay(CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);
        esp_restart();

    ret:
        // OTA_COMPLETE_EVENT with err
        esp_event_post_to(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, &err, sizeof(esp_err_t), portMAX_DELAY);

        esp_ota_abort(update_handle);
        http_cleanup(client);
        // disconnect wifi and unregister event handler
        wifi_disconnect();
        vTaskDelete(NULL);
    }

    static void
    global_system_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
    {
        EventGroupHandle_t wifi_event_group = *(EventGroupHandle_t *)handler_arg;
        // 如果是Wi-Fi事件，并且事件ID是Wi-Fi事件STA_START
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(TAG_FIRMWARE, "Fail to connect");
        }
        else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
        {
            // 如果是IP事件，并且事件ID是IP事件STA_GOT_IP
            // 获取事件结果
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG_FIRMWARE, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));

            // 通过调用 xEventGroupSetBits 函数，将 WIFI_CONNECTED_BIT 设置到事件组中，表示成功连接到 AP
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }

    static void
    global_pr_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
    {
        if (event_base == PRC)
        {
            switch (event_id)
            {
            case OTA_COMPLETE_EVENT:
            {
                esp_err_t *err = (esp_err_t *)event_data;
                if (*err == ESP_OK)
                {
                    ESP_LOGI(TAG_FIRMWARE, "OTA Complete");
                    vTaskDelay(CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);
                }
                else if (*err == ESP_ERR_NOT_SUPPORTED)
                {
                    ESP_LOGW(TAG_FIRMWARE, "OTA Skipped (%s)", esp_err_to_name(*err));
                }
                else
                {
                    ESP_LOGE(TAG_FIRMWARE, "OTA Failed (%s)", esp_err_to_name(*err));
                }
                xEventGroupSetBits((EventGroupHandle_t)handler_arg, OTA_COMPLETE_BIT);
                break;
            }
            case OTA_BEGIN_EVENT:
            {

                ESP_LOGI(TAG_FIRMWARE, "OTA Triggered");
                break;
            }
            default:
                break;
            }
        }
    }

    ~Firmware()
    {
        wifi_disconnect();

        if (ota_task_handle != NULL)
        {
            vTaskDelete(ota_task_handle);
        }
    }
};

// Firmware class static variable
esp_app_desc_t Firmware::app_desc = {0};
esp_netif_t *Firmware::netif = NULL;
