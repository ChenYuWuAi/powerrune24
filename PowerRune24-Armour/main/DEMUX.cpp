#include "DEMUX.h"
DEMUX::DEMUX(const gpio_num_t *DEMUX_addr_IO, gpio_num_t DEMUX_IO_enable, uint8_t chan, uint8_t enable)
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

esp_err_t DEMUX::enable()
{
    this->enable_state = 1;
    gpio_set_level(this->DEMUX_IO_enable, !this->enable_state); // 低电平有效
    return ESP_OK;
}

esp_err_t DEMUX::disable()
{
    this->enable_state = 0;
    gpio_set_level(this->DEMUX_IO_enable, !this->enable_state); // 低电平有效
    return ESP_OK;
}

esp_err_t DEMUX::set_channel(uint8_t channel)
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

uint8_t DEMUX::get_state()
{
    return this->enable_state;
}

uint8_t DEMUX::get_channel()
{
    return this->channel;
}

uint8_t &DEMUX::operator=(uint8_t channel)
{
    this->set_channel(channel);
    return this->channel;
}

DEMUX::~DEMUX()
{
    gpio_reset_pin(this->DEMUX_IO_enable);
    for (uint8_t i = 0; i < BITS; i++)
    {
        gpio_reset_pin(this->DEMUX_IO[i]);
    }
}
