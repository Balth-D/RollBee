#include "BoardOutput.hpp"

#include "driver/gpio.h"

BoardOutput::BoardOutput(gpio_num_t pin_id, gpio_mode_t mode)
    : pin_id{pin_id}
    , cached_state{false}
{
    gpio_reset_pin(pin_id);
    gpio_set_direction(pin_id, mode);
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
