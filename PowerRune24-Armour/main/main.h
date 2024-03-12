#include <stdio.h>
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>
#include "LED.h"
#include <driver/gpio.h>
#include "firmware.h"
#include "PowerRune_Armour.h"

extern Config *config;
extern LED *led;
extern esp_event_loop_handle_t pr_events_loop_handle;

// ESP Protocol Indicator
ESPNowProtocol *espnow_protocol;
