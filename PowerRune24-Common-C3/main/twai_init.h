/*
File Name:      twai.h

Function List:  twai_init()
*/

#ifndef _TWAI_H_
#define _TWAI_H_

#ifdef  __cplusplus
extern  "C"  {
#endif

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <hal/twai_types.h>
#include <esp_log.h>

//#define TX_TWAI_PIN  GPIO_NUM_4
//#define RX_TWAI_PIN  GPIO_NUM_5

static const twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static TaskHandle_t TWAI_handle;

void twai_init(gpio_num_t TX_TWAI_PIN, gpio_num_t RX_TWAI_PIN) {
    

    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_TWAI_PIN, RX_TWAI_PIN, TWAI_MODE_NORMAL);

    if (twai_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK) {
        ESP_LOGE("TWAI", "Failed to install TWAI driver");
    }

    if (twai_start() != ESP_OK) {
        ESP_LOGE("TWAI", "Failed to start TWAI driver.");
    }

    ESP_LOGI("TWAI", "TWAI initialized.");
}

#ifdef  __cplusplus
}
#endif

#endif /* _TWAI_H_ */