#include "BoardInput.hpp"

#include "driver/gpio.h"

BoardInput::BoardInput(gpio_num_t pin_id, gpio_mode_t mode)
    : pin_id{pin_id}
{
    gpio_config_t cfg = {
        .pin_bit_mask           = BIT64(pin_id),
        .mode                   = mode,
        .pull_up_en             = GPIO_PULLUP_DISABLE,
        .pull_down_en           = GPIO_PULLDOWN_DISABLE,
        .intr_type              = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

bool BoardInput::Read(void) const
{
    return gpio_get_level(pin_id);
}

bool BoardInput::EnableInterrupt(gpio_int_type_t edge)
{
    GlobalInterruptSetup();

    if (gpio_isr_handler_add(pin_id, GlobalInterruptHandler, this) == ESP_OK
        && gpio_set_intr_type(pin_id, edge) == ESP_OK
        && gpio_intr_enable(pin_id) == ESP_OK)
    {
        return true;
    }

    return false;
}

bool BoardInput::DisableInterrupt(void)
{
    return (gpio_intr_disable(pin_id) == ESP_OK);
}

bool BoardInput::SetInterruptCallback(BoardInputCallback cb, void* data)
{
    if (cb != nullptr)
    {
        interrupt_callback = cb;
        interrupt_callback_data = data;

        return true;
    }

    return false;
}

void BoardInput::GlobalInterruptSetup(void)
{
    static bool init_done = false;

    if (!init_done)
    {
        gpio_install_isr_service(0);
        init_done = true;
    }
}

void BoardInput::GlobalInterruptHandler(void* data)
{
    if (data == nullptr) { return; }

    BoardInput* this_ptr = static_cast<BoardInput*>(data);

    if (this_ptr->interrupt_callback != nullptr)
    {
        this_ptr->interrupt_callback(this_ptr->interrupt_callback_data);
    }
}
