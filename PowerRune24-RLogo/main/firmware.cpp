/**
 * @file firmware.cpp
 * @brief 固件类
 * @version 0.6
 * @date 2024-02-19
 */
#include "firmware.h"

// LOG TAG
static const char *TAG_FIRMWARE = "Firmware";

// Config class variable
Config *config = NULL;
esp_event_loop_handle_t pr_events_loop_handle = NULL;
// Firmware class static variable
esp_app_desc_t Firmware::app_desc = {0};
esp_netif_t *Firmware::netif = NULL;
EventGroupHandle_t Firmware::ota_event_group = NULL;

#if CONFIG_POWERRUNE_TYPE == 0 // ARMOUR
const char *Config::PowerRune_description = "Armour";
PowerRune_Armour_config_info_t Config::config_info = {0};
#endif
QueueHandle_t Firmware::ota_complete_queue = NULL;
#if CONFIG_POWERRUNE_TYPE == 1
const char *Config::PowerRune_description = "RLogo";
PowerRune_Rlogo_config_info_t Config::config_info = {0};
PowerRune_Armour_config_info_t Config::config_armour_info[5] = {0};
PowerRune_Motor_config_info_t Config::config_motor_info = {0};
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // MOTORCTL
const char *Config::PowerRune_description = "Motor";
PowerRune_Motor_config_info_t Config::config_info = {0};
#endif
PowerRune_Common_config_info_t Config::config_common_info = {0};

Config::Config()
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
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // ARMOUR || MOTORCTL
    esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT, (esp_event_handler_t)Config::global_event_handler, this);
#endif
    ESP_LOGI(TAG_FIRMWARE, "Configuration Complete");
}

// 获取数据指针
#if CONFIG_POWERRUNE_TYPE == 0 // ARMOUR
const PowerRune_Armour_config_info_t *Config::get_config_info_pt()
{
    return &config_info; // Read config_common_info from NVS
}
#endif
#if CONFIG_POWERRUNE_TYPE == 1
PowerRune_Rlogo_config_info_t *Config::get_config_info_pt()
{ // Read config_common_info from NVS

    return &config_info;
}

PowerRune_Armour_config_info_t *Config::get_config_armour_info_pt(uint8_t index)
{
    return &config_armour_info[index];
}

PowerRune_Motor_config_info_t *Config::get_config_motor_info_pt()
{
    return &config_motor_info;
}

PowerRune_Common_config_info_t *Config::get_config_common_info_pt()
{
    return &config_common_info;
}
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // MOTORCTL
const PowerRune_Motor_config_info_t *Config::get_config_info_pt()
{
    return &config_info;
}
#endif
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // ARMOUR || MOTORCTL
const PowerRune_Common_config_info_t *Config::get_config_common_info_pt()
{
    return &config_common_info;
}
#endif

esp_err_t Config::read()
{
    esp_err_t ret = ESP_OK;
    // Read config_info from NVS
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READWRITE, &my_handle));
// BLOB
#if CONFIG_POWERRUNE_TYPE == 0 // ARMOUR
    size_t required_size = sizeof(PowerRune_Armour_config_info_t);
#endif
#if CONFIG_POWERRUNE_TYPE == 1
    size_t required_size = sizeof(PowerRune_Rlogo_config_info_t);
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // MOTORCTL
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

esp_err_t Config::save()
{
    // Write config_info to NVS
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READWRITE, &my_handle));
// BLOB
#if CONFIG_POWERRUNE_TYPE == 0 // ARMOUR
    size_t required_size = sizeof(PowerRune_Armour_config_info_t);
#endif
#if CONFIG_POWERRUNE_TYPE == 1 // RLOGO
    size_t required_size = sizeof(PowerRune_Rlogo_config_info_t);
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // MOTORCTL
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

esp_err_t Config::reset()
{
#if CONFIG_POWERRUNE_TYPE == 0 // ARMOUR
    config_info = {
        .brightness = 127,
        .armour_id = 0xFF,
        .brightness_proportion_matrix = 127,
        .brightness_proportion_edge = 127,
    };
#endif
#if CONFIG_POWERRUNE_TYPE == 1
    config_info = {
        .brightness = 127,
    };
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // MOTORCTL
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
        .URL = CONFIG_DEFAULT_UPDATE_URL, // 需要往后加上CONFIG_DEFAULT_UPDATE_FILE
        .SSID = CONFIG_DEFAULT_UPDATE_SSID,
        .SSID_pwd = CONFIG_DEFAULT_UPDATE_PWD,
        .auto_update = 1,
    };
    Config::save();
    return ESP_OK;
}

void Config::global_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_err_t err = ESP_OK;
    if (id == CONFIG_EVENT)
    {
        CONFIG_EVENT_DATA *config_event_data = (CONFIG_EVENT_DATA *)event_data;
        memcpy(&config_common_info, &config_event_data->config_common_info, sizeof(PowerRune_Common_config_info_t));
// event_data: PowerRune_Common_config_info_t, PowerRune_Armour/RLogo/Motor_config_info_t
#if CONFIG_POWERRUNE_TYPE == 0 // ARMOUR
        memcpy(&config_info, &config_event_data->config_armour_info, sizeof(PowerRune_Armour_config_info_t));
#endif
#if CONFIG_POWERRUNE_TYPE == 1 // RLOGO
        memcpy(&config_info, &config_event_data->config_rlogo_info, sizeof(PowerRune_Rlogo_config_info_t));
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // MOTORCTL
        memcpy(&config_info, &config_event_data->config_motor_info, sizeof(PowerRune_Motor_config_info_t));
#endif
        // Write config_info to NVS
        err = Config::save();
        CONFIG_COMPLETE_EVENT_DATA config_complete_event_data;
        config_complete_event_data.status = err;
        // Post Complete Event
        esp_event_post_to(pr_events_loop_handle, PRC, CONFIG_COMPLETE_EVENT, &config_complete_event_data, sizeof(CONFIG_COMPLETE_EVENT_DATA), portMAX_DELAY);
    }
}

esp_err_t Firmware::wifi_ota_init()
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
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); // BLE共存
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_ps(WIFI_PS_NONE);
    return ESP_OK;
}

// Firmware Functions
esp_err_t Firmware::wifi_connect(const PowerRune_Common_config_info_t *config_common_info, uint8_t retryNum)
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
                .authmode = strlen(config_common_info->SSID_pwd) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
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
    return ESP_OK;
}

void Firmware::wifi_disconnect()
{
    // 断开连接
    ESP_ERROR_CHECK(esp_wifi_disconnect());
}

void Firmware::http_cleanup(esp_http_client_handle_t client)
{
    if (client == NULL)
        return;
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

Firmware::Firmware()
{ // 常亮LED
    led->set_mode(LED_MODE_ON, 1);
    // OTA Event Group
    ota_event_group = xEventGroupCreate();

    uint8_t ota_restart = 1;
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
    wifi_ota_init();
    // 注册事件处理器
    esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, (esp_event_handler_t)Firmware::global_pr_event_handler, NULL);
    if (config->get_config_common_info_pt()->auto_update)
    {
        // Start OTA
        ESP_LOGI(TAG_FIRMWARE, "Auto update enabled. Starting OTA task...");

        // led闪烁
        led->set_mode(LED_MODE_BLINK, 0);
        xTaskCreate((TaskFunction_t)Firmware::task_OTA, "OTA", 8192, &ota_restart, 5, NULL);

        xEventGroupWaitBits(ota_event_group, OTA_COMPLETE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
        // led常亮
        led->set_mode(LED_MODE_ON, 1);
    }
    ESP_LOGI(TAG_FIRMWARE, "Firmware init complete");
}

void Firmware::task_OTA(void *args)
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
    OTA_COMPLETE_EVENT_DATA ota_complete_event_data;

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
        .skip_cert_common_name_check = true,
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
    ota_complete_event_data.status = wifi_connect(config_common_info, 5); // Retry 3 times
    if (ota_complete_event_data.status != ESP_OK)
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
    ota_complete_event_data.status = esp_http_client_open(client, 0);
    if (ota_complete_event_data.status != ESP_OK)
    {
        ESP_LOGE(TAG_FIRMWARE, "Failed to open HTTP connection: %s", esp_err_to_name(ota_complete_event_data.status));
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
                    ESP_LOGI(TAG_FIRMWARE, "Server firmware version: %s", new_app_info.version);

                    const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                    {
                        ESP_LOGI(TAG_FIRMWARE, "Last invalid firmware version: %s", invalid_app_info.version);
                    }
                    // version check
                    if (last_invalid_app != NULL)
                    {
                        // 版本新旧比较
                        if (memcmp(new_app_info.version, invalid_app_info.version, sizeof(new_app_info.version)) <= 0)
                        {
                            ota_complete_event_data.status = ESP_ERR_NOT_SUPPORTED;
                            ESP_LOGW(TAG_FIRMWARE, "Older or same version of last invalid firmware is running. Update refused.");
                            goto ret;
                        }
                    }
                    if (memcmp(new_app_info.version, Firmware::app_desc.version, sizeof(new_app_info.version)) <= 0)
                    {
                        ESP_LOGW(TAG_FIRMWARE, "Older or same version of running firmware is running. Skipped.");
                        ota_complete_event_data.status = ESP_ERR_NOT_SUPPORTED;
                        goto ret;
                    }
                    image_header_was_checked = true;
                    ESP_LOGI(TAG_FIRMWARE, "Target partition subtype %d at offset 0x%" PRIx32,
                             update_partition->subtype, update_partition->address);
                    ota_complete_event_data.status = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (ota_complete_event_data.status != ESP_OK)
                    {
                        ESP_LOGE(TAG_FIRMWARE, "esp_ota_begin failed (%s)", esp_err_to_name(ota_complete_event_data.status));
                        goto ret;
                    }
                    ESP_LOGI(TAG_FIRMWARE, "Downloading and flashing firmware...");
                }
                else
                {
                    ESP_LOGE(TAG_FIRMWARE, "received package is not fit len");
                    ota_complete_event_data.status = ESP_ERR_OTA_VALIDATE_FAILED;
                    goto ret;
                }
            }

            ota_complete_event_data.status = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
            if (ota_complete_event_data.status != ESP_OK)
            {
                ESP_LOGE(TAG_FIRMWARE, "esp_ota_write failed (%s)", esp_err_to_name(ota_complete_event_data.status));
                esp_ota_abort(update_handle);
                goto ret;
            }
            read_length += data_read;
            count++;
            if (count % 10 == 0)
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
        ota_complete_event_data.status = ESP_ERR_OTA_VALIDATE_FAILED;
        goto ret;
    }
    // Verify firmware
    ota_complete_event_data.status = esp_ota_end(update_handle);
    if (ota_complete_event_data.status != ESP_OK)
    {
        if (ota_complete_event_data.status == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            ESP_LOGE(TAG_FIRMWARE, "Image validation failed, image is corrupted");
        }
        else
        {
            ESP_LOGE(TAG_FIRMWARE, "esp_ota_end failed (%s)!", esp_err_to_name(ota_complete_event_data.status));
        }
        goto ret;
    }
    ota_complete_event_data.status = esp_ota_set_boot_partition(update_partition);
    if (ota_complete_event_data.status != ESP_OK)
    {
        ESP_LOGE(TAG_FIRMWARE, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(ota_complete_event_data.status));
        goto ret;
    }

    // post OTA_COMPLETE_EVENT and wait for ack
    ESP_LOGI(TAG_FIRMWARE, "OTA complete, ready to restart");
    http_cleanup(client);
    // disconnect wifi and unregister event handler
    wifi_disconnect();
    // 视情况重启ESP_NOW
    if (*((uint8_t *)args) >= 2)
    {
        do
        {
            err = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
        } while (err == ESP_FAIL);
    }
    ota_complete_event_data.status = ESP_OK;
    ota_complete_event_data.ota_type = *((uint8_t *)args);
    esp_event_post_to(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, &ota_complete_event_data, sizeof(OTA_COMPLETE_EVENT_DATA), portMAX_DELAY);
#if CONFIG_POWERRUNE_TYPE != 1
    // 如果是开机自动更新，则忽略
    if (*((uint8_t *)args) != 1)
    {
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }
#endif
    if (*((uint8_t *)args) >= 1) // Restart enable
    {
        esp_restart();
    }

ret:
    esp_ota_abort(update_handle);
    http_cleanup(client);
    // disconnect wifi and unregister event handler
    wifi_disconnect();
    // 视情况重启ESP_NOW
    if (*((uint8_t *)args) >= 2)
    {
        do
        {
            err = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
        } while (err == ESP_FAIL);
    }
    // OTA_COMPLETE_EVENT with err
    ota_complete_event_data.ota_type = *((uint8_t *)args);
    esp_event_post_to(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, &ota_complete_event_data, sizeof(OTA_COMPLETE_EVENT_DATA), portMAX_DELAY);
#if CONFIG_POWERRUNE_TYPE != 1
    // 如果是开机自动更新，则忽略
    if (*(uint8_t *)args != 1)
    {
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }
#endif
    vTaskDelete(NULL);
}

void Firmware::global_system_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
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
        ESP_LOGI(TAG_FIRMWARE, "IP:" IPSTR, IP2STR(&event->ip_info.ip));

        // 通过调用 xEventGroupSetBits 函数，将 WIFI_CONNECTED_BIT 设置到事件组中，表示成功连接到 AP
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void Firmware::global_pr_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{

    if (event_base == PRC)
    {
        switch (event_id)
        {
        case OTA_COMPLETE_EVENT:
        {
            OTA_COMPLETE_EVENT_DATA *ota_complete_event_data = (OTA_COMPLETE_EVENT_DATA *)event_data;
            esp_log_buffer_hex(TAG_FIRMWARE, ota_complete_event_data, sizeof(OTA_COMPLETE_EVENT_DATA));
            if (ota_complete_event_data->status == ESP_OK)
            {
                ESP_LOGI(TAG_FIRMWARE, "Device %i OTA Complete", ota_complete_event_data->address);
            }
            else if (ota_complete_event_data->status == ESP_ERR_NOT_SUPPORTED)
            {
                ESP_LOGW(TAG_FIRMWARE, "Device %i OTA Skipped (%s)", ota_complete_event_data->address, esp_err_to_name(ota_complete_event_data->status));
            }
            else
            {
                ESP_LOGE(TAG_FIRMWARE, "Device %i OTA Failed (%s)", ota_complete_event_data->address, esp_err_to_name(ota_complete_event_data->status));
            }
#if CONFIG_POWERRUNE_TYPE == 1
            // 将status通过FreeRTOS任务通知发送给ota_task
            if (xEventGroupGetBits(ota_event_group) & OTA_COMPLETE_LISTENING_BIT)
            {
                assert(ota_complete_queue != NULL);
                xQueueSend(ota_complete_queue, ota_complete_event_data, portMAX_DELAY);
            }
#else
            // 如果是手动更新，则重启beacon，打开呼吸灯
            if (ota_complete_event_data->ota_type != 1)
            {
                ESPNowProtocol::beacon_control(1);
                led->set_mode(LED_MODE_FADE, 0);
            }

#endif

            xEventGroupSetBits(Firmware::ota_event_group, Firmware::OTA_COMPLETE_BIT);
            break;
        }
        case OTA_BEGIN_EVENT:
        {
            OTA_BEGIN_EVENT_DATA *ota_begin_event_data = (OTA_BEGIN_EVENT_DATA *)event_data;
// begin ota task
#if CONFIG_POWERRUNE_TYPE == 1
            assert(ota_complete_queue != NULL);
            if (ota_begin_event_data->address != 0x06)
                return;
            ESP_LOGI(TAG_FIRMWARE, "OTA Triggered, starting OTA task...");
            uint8_t ota_restart = 0; // 表示不重启
#else
            ESPNowProtocol::beacon_control(0);
            uint8_t ota_restart = 2; // 表示重设ESP_NOW通道，回复更新状态后，重启
            ESP_LOGI(TAG_FIRMWARE, "OTA Triggered, starting OTA task...");
#endif

            led->set_mode(LED_MODE_BLINK, 0);
            xTaskCreate((TaskFunction_t)Firmware::task_OTA, "OTA", 8192, &ota_restart, 5, NULL);
            break;
        }
        default:
            break;
        }
    }
}
