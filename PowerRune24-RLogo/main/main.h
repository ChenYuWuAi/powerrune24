/**
 * @file "main.h"
 * @note main.cpp声明
 */
#pragma once
#include <stdio.h>

// led
#include "LED_Strip.h"
#include "LED.h"

// Common
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "string.h"
#include "esp_random.h"
#include <vector>

// BLE
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// Messager
#include "espnow_protocol.h"

// Firmware
#include "firmware.h"

// LOG TAG
static const char *TAG_MAIN = "Main";

// LED_Strip
LED_Strip LED_Strip_0(GPIO_NUM_10, 49);
// led animation task handle
TaskHandle_t led_animation_task_handle = NULL;
// LED Indicator
extern LED *led;
// LED Strip Indicator
LED_Strip *led_strip = NULL;
// ESP Protocol Indicator
ESPNowProtocol *espnow_protocol;
// Config Class
extern Config *config;
extern esp_event_loop_handle_t pr_events_loop_handle;

// Score Vector
std::vector<uint8_t> score_vector;

QueueHandle_t run_queue;
// FreeRTOS计时器
TimerHandle_t hit_timer;
SemaphoreHandle_t motor_done_sem;

void run_task(void *pvParameter);
void ota_task(void *pvParameter);
void beacon_timeout(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

// event loop
#include "ble_events.h"
#include "ble_handlers.h"

#ifdef HEAPDEBUG
#include "esp_heap_caps.h"
#endif