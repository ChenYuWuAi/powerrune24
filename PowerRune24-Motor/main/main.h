#include <stdio.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_event_base.h>

#include "PowerRune_Events.h"
#include "motor_ctrl.h"
#include "LED.h"
#include "firmware.h"
#include "espnow_protocol.h"


// event loop TAG
static const char *TAG = "PRM";

// LED Indicator
extern LED *led;
// ESP Protocol Indicator
ESPNowProtocol *espnow_protocol;
// Config Class
extern Config *config;
extern esp_event_loop_handle_t pr_events_loop_handle;