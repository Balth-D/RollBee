#include "BoardOutput.hpp"

#include "driver/gpio.h"

BoardOutput::BoardOutput(gpio_num_t pin_id, gpio_mode_t mode, bool default_state)
    : pin_id{pin_id}
    , cached_state{false}
{
    gpio_config_t cfg = {
        .pin_bit_mask           = BIT64(pin_id),
        .mode                   = mode,
        .pull_up_en             = GPIO_PULLUP_DISABLE,
        .pull_down_en           = GPIO_PULLDOWN_DISABLE,
        .intr_type              = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    Write(default_state);
}

bool BoardOutput::Read(void) const
{
    return cached_state;
}

bool BoardOutput::Write(bool state)
{
    gpio_set_level(pin_id, state);
    cached_state = state;

    return true;
}

bool BoardOutput::Toggle(void)
{
    return Write(!cached_state);
}
