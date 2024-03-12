/**
 * @file DEMUX.h
 * @brief DEMUX驱动
 * @version 0.1
 * @date 2024-01-24
 * @author CH
 */
#pragma once
#ifndef _DEMUX_H_
#define _DEMUX_H_
#include "driver/gpio.h"
#define BITS 3
class DEMUX
{
private:
    gpio_num_t DEMUX_IO[BITS]; // LSB -> MSB
    gpio_num_t DEMUX_IO_enable;
    uint8_t channel;
    uint8_t enable_state;
    
public:
    DEMUX(const gpio_num_t *DEMUX_IO, gpio_num_t DEMUX_IO_enable, uint8_t chan = 0, uint8_t enable = 0);
    esp_err_t enable();
    esp_err_t disable();
    esp_err_t set_channel(uint8_t channel);
    uint8_t get_state();
    uint8_t get_channel();
    uint8_t &operator=(uint8_t channel);
    ~DEMUX();
};
#endif