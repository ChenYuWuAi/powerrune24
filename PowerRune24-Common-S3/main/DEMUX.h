/**
 * @file DEMUX.h
 * @brief DEMUX驱动
 * @version 0.1
 * @date 2024-01-24
 * @author CH
 */
#include "driver/gpio.h"
template <uint8_t BITS>
class DEMUX
{
private:
    gpio_num_t DEMUX_IO[BITS]; // LSB -> MSB
    gpio_num_t DEMUX_IO_enable;
    uint8_t channel;
    uint8_t enable_state;

public:
    DEMUX(gpio_num_t *DEMUX_IO, gpio_num_t DEMUX_IO_enable, uint8_t chan = 0, uint8_t enable = 0);
    esp_err_t enable();
    esp_err_t disable();
    esp_err_t set_channel(uint8_t channel);
    uint8_t get_state();
    uint8_t get_channel();
    uint8_t &operator=(uint8_t channel);
    ~DEMUX();
};

template <uint8_t BITS>
DEMUX<BITS>::DEMUX(gpio_num_t *DEMUX_addr_IO, gpio_num_t DEMUX_IO_enable, uint8_t chan, uint8_t enable)
{
    this->DEMUX_IO_enable = DEMUX_IO_enable;
    this->channel = chan;
    this->enable_state = enable;
    for (uint8_t i = 0; i < BITS; i++)
    {
        this->DEMUX_IO[i] = DEMUX_addr_IO[i];
    }
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 0;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    for (uint8_t i = 0; i < BITS; i++)
    {
        io_conf.pin_bit_mask |= (1ULL << this->DEMUX_IO[i]);
    }
    io_conf.pin_bit_mask |= (1ULL << this->DEMUX_IO_enable);
    gpio_config(&io_conf);
    gpio_set_level(this->DEMUX_IO_enable, this->enable_state);
    this->set_channel(this->channel);
}

template <uint8_t BITS>
esp_err_t DEMUX<BITS>::enable()
{
    this->enable_state = 1;
    gpio_set_level(this->DEMUX_IO_enable, !this->enable_state); // 低电平有效
    return ESP_OK;
}

template <uint8_t BITS>
esp_err_t DEMUX<BITS>::disable()
{
    this->enable = 0;
    gpio_set_level(this->DEMUX_IO_enable, !this->enable); // 低电平有效
    return ESP_OK;
}

template <uint8_t BITS>
esp_err_t DEMUX<BITS>::set_channel(uint8_t channel)
{
    if (channel > (1 << BITS) - 1)
    {
        return ESP_ERR_INVALID_ARG;
    }
    this->channel = channel;
    for (uint8_t i = 0; i < BITS; i++)
    {
        gpio_set_level(this->DEMUX_IO[i], (this->channel >> i) & 0x01);
    }
    return ESP_OK;
}

template <uint8_t BITS>
uint8_t DEMUX<BITS>::get_state()
{
    return this->enable;
}

template <uint8_t BITS>
uint8_t DEMUX<BITS>::get_channel()
{
    return this->channel;
}

template <uint8_t BITS>
uint8_t &DEMUX<BITS>::operator=(uint8_t channel)
{
    this->set_channel(channel);
    return this->channel;
}

template <uint8_t BITS>
DEMUX<BITS>::~DEMUX()
{
    gpio_reset_pin(this->DEMUX_IO_enable);
    for (uint8_t i = 0; i < BITS; i++)
    {
        gpio_reset_pin(this->DEMUX_IO[i]);
    }
}
