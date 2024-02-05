/**
 * @file "main.h"
 * @note main.cpp声明
 */
#pragma once
#include <stdio.h>

// led
#include "LED_Strip.h"
#include "LED.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

// Firmware
#include "firmware.h"



// LED_Strip
LED_Strip LED_Strip_0(GPIO_NUM_10, 49);
// LED Indicator
extern LED *led;
// Config Class
extern Config *config;
extern esp_event_loop_handle_t pr_events_loop_handle;

void run_task(void *pvParameter);
void ota_task(void *pvParameter);

// event loop
#include "ble_events.h"
#include "ble_handlers.h"