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

// ESP-NOW
#include "esp_now.h"

#define OTA_BUF_SIZE 512 // OTA缓冲区大小 单位：字节

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

        // Read config_info from NVS
        err = read();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_FIRMWARE, "read config_info from NVS failed (%s)", esp_err_to_name(err));
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
    enum PR_type // PowerRune Type
    {
        ARMOUR,
        RLOGO,
        MOTORCTL,
    };

    struct board_info_t
    {
        PR_type board_type;
        uint8_t board_id; // 仅对于ARMOUR有效 range[1:5]
        char version[25]; // 程序版本号，从esp_app_desc_t中获取
        Config<PowerRune_config_info_t> config;
    } board_info;

public:
    // 设置板子类型 和 OTA
    Firmware()
    {
        // 上电初始化逻辑

        // 读取版本号
        esp_app_desc_t app_desc;
        esp_err_t err = esp_ota_get_partition_description(esp_ota_get_running_partition(), &app_desc);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_FIRMWARE, "esp_ota_get_partition_description failed (%s)", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG_FIRMWARE, "Running firmware version: %s", app_desc.version);
        }
        board_info.version = app_desc.version;
// 读取板子类型
#if CONFIG_POWER_RUNE_TYPE == 0 // 从menuconfig中读取
        board_info.board_type = ARMOUR;
        // 读取板子ID
        board_info.board_id = config_info.armour_id;
#elif CONFIG_POWER_RUNE_TYPE == 1
        board_info.board_type = RLOGO;
        board_info.board_id = 0; // 无效
#elif CONFIG_POWER_RUNE_TYPE == 2
        board_info.board_type = MOTORCTL;
        board_info.board_id = 0; // 无效
#endif
        board_info.config = config_info;

        // 使能Wifi，为OTA和ESP-NOW做准备
        wifi_init_sta();

        // 自动更新
        if (config_common_info.auto_update)
        {
            // OTA Task
            xTaskCreate(&global_task_OTA, "OTA Task", 8192, this, 5, NULL);
        }
    }

    esp_err_t wifi_init_sta()
    {
        // Wifi Init
        ESP_LOGI(TAG_FIRMWARE, "ESP_WIFI_MODE_STA");
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        esp_wifi_set_ps(WIFI_PS_NONE); // Prevent WiFi from sleeping
        wifi_config_t wifi_config = {
            .sta = {
                .ssid = config_common_info.SSID,
                .password = config_common_info.SSID_pwd,
            },
        };
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG_FIRMWARE, "wifi_init_sta finished.");
        // Wait for connection
        uint8_t retry = 0;
        while (retry < 10)
        {
            if (esp_wifi_sta_get_connect_status() == WIFI_CONNECTED)
            {
                ESP_LOGI(TAG_FIRMWARE, "connected to ap SSID:%s",
                         config_common_info.SSID);
                return ESP_OK;
            }
            ESP_LOGI(TAG_FIRMWARE, "Waiting for connection to the wifi network... (%d/10)", retry);
            retry++;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        return ESP_FAIL;
    }

    esp_err_t task_OTA()
    {
        // Wifi Init
        ESP_ERROR_CHECK(wifi_init_sta());
        // Use Native OTA API
        esp_err_t err;
        /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
        esp_ota_handle_t update_handle = 0;
        const esp_partition_t *update_partition = NULL;

        ESP_LOGI(TAG, "Starting OTA example task");

        const esp_partition_t *configured = esp_ota_get_boot_partition();
        const esp_partition_t *running = esp_ota_get_running_partition();

        if (configured != running)
        {
            ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08" PRIx32 ", but running from offset 0x%08" PRIx32,
                     configured->address, running->address);
            ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
        }
        ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08" PRIx32 ")",
                 running->type, running->subtype, running->address);

        esp_http_client_config_t config = {
            .url = CONFIG_EXAMPLE_FIRMWARE_UPG_URL,
            .cert_pem = (char *)server_cert_pem_start,
            .timeout_ms = CONFIG_EXAMPLE_OTA_RECV_TIMEOUT,
            .keep_alive_enable = true,
        };

#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
        char url_buf[OTA_URL_SIZE];
        if (strcmp(config.url, "FROM_STDIN") == 0)
        {
            example_configure_stdin_stdout();
            fgets(url_buf, OTA_URL_SIZE, stdin);
            int len = strlen(url_buf);
            url_buf[len - 1] = '\0';
            config.url = url_buf;
        }
        else
        {
            ESP_LOGE(TAG, "Configuration mismatch: wrong firmware upgrade image url");
            abort();
        }
#endif

#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
        config.skip_cert_common_name_check = true;
#endif

        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client == NULL)
        {
            ESP_LOGE(TAG, "Failed to initialise HTTP connection");
            task_fatal_error();
        }
        err = esp_http_client_open(client, 0);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            task_fatal_error();
        }
        esp_http_client_fetch_headers(client);

        update_partition = esp_ota_get_next_update_partition(NULL);
        assert(update_partition != NULL);
        ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%" PRIx32,
                 update_partition->subtype, update_partition->address);

        int binary_file_length = 0;
        /*deal with all receive packet*/
        bool image_header_was_checked = false;
        while (1)
        {
            int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
            if (data_read < 0)
            {
                ESP_LOGE(TAG, "Error: SSL data read error");
                http_cleanup(client);
                task_fatal_error();
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
                        ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                        esp_app_desc_t running_app_info;
                        if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                        {
                            ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                        }

                        const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                        esp_app_desc_t invalid_app_info;
                        if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                        {
                            ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                        }

                        // check current version with last invalid partition
                        if (last_invalid_app != NULL)
                        {
                            if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                            {
                                ESP_LOGW(TAG, "New version is the same as invalid version.");
                                ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                                ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                                http_cleanup(client);
                                infinite_loop();
                            }
                        }
#ifndef CONFIG_EXAMPLE_SKIP_VERSION_CHECK
                        if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                        {
                            ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                            http_cleanup(client);
                            infinite_loop();
                        }
#endif

                        image_header_was_checked = true;

                        err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                            http_cleanup(client);
                            esp_ota_abort(update_handle);
                            task_fatal_error();
                        }
                        ESP_LOGI(TAG, "esp_ota_begin succeeded");
                    }
                    else
                    {
                        ESP_LOGE(TAG, "received package is not fit len");
                        http_cleanup(client);
                        esp_ota_abort(update_handle);
                        task_fatal_error();
                    }
                }
                err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
                if (err != ESP_OK)
                {
                    http_cleanup(client);
                    esp_ota_abort(update_handle);
                    task_fatal_error();
                }
                binary_file_length += data_read;
                ESP_LOGD(TAG, "Written image length %d", binary_file_length);
            }
            else if (data_read == 0)
            {
                /*
                 * As esp_http_client_read never returns negative error code, we rely on
                 * `errno` to check for underlying transport connectivity closure if any
                 */
                if (errno == ECONNRESET || errno == ENOTCONN)
                {
                    ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
                    break;
                }
                if (esp_http_client_is_complete_data_received(client) == true)
                {
                    ESP_LOGI(TAG, "Connection closed");
                    break;
                }
            }
        }
        ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);
        if (esp_http_client_is_complete_data_received(client) != true)
        {
            ESP_LOGE(TAG, "Error in receiving complete file");
            http_cleanup(client);
            esp_ota_abort(update_handle);
            task_fatal_error();
        }

        err = esp_ota_end(update_handle);
        if (err != ESP_OK)
        {
            if (err == ESP_ERR_OTA_VALIDATE_FAILED)
            {
                ESP_LOGE(TAG, "Image validation failed, image is corrupted");
            }
            else
            {
                ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
            }
            http_cleanup(client);
            task_fatal_error();
        }

        err = esp_ota_set_boot_partition(update_partition);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
            http_cleanup(client);
            task_fatal_error();
        }
        ESP_LOGI(TAG, "Prepare to restart system!");
        esp_restart();
        return;
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