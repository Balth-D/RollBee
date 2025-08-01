#pragma once

#include <stdint.h>
#include "soc/gpio_num.h"
#include "hal/gpio_types.h"

class BoardInput
{
public:
    typedef void (*BoardInputCallback)(void*);

    BoardInput(gpio_num_t pin_id, gpio_mode_t mode);

    bool Read(void) const;

    bool EnableInterrupt(gpio_int_type_t edge);
    bool DisableInterrupt(void);
    bool SetInterruptCallback(BoardInputCallback cb, void* data);

private:
    const gpio_num_t pin_id;
    void*               interrupt_callback_data = nullptr;
    BoardInputCallback  interrupt_callback      = nullptr;

    static void GlobalInterruptSetup(void);
    static void GlobalInterruptHandler(void* data);

};
