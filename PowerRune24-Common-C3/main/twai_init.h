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
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <hal/twai_types.h>
#include <esp_log.h>

#define TWAI_TX_PIN  GPIO_NUM_4
#define TWAI_RX_PIN  GPIO_NUM_5

static const twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

void twai_init() {
    
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);

    if (twai_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK) {
        ESP_LOGE("TWAI", "Failed to install TWAI driver");
    }

    if (twai_start() != ESP_OK) {
        ESP_LOGE("TWAI", "Failed to start TWAI driver");
    }

    ESP_LOGI("TWAI", "TWAI initialized");
}

#ifdef  __cplusplus
}
#endif

#endif /* _TWAI_H_ */