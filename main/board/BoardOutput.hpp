#pragma once

#include <stdint.h>
#include "soc/gpio_num.h"
#include "hal/gpio_types.h"

class BoardOutput
{
public:
    BoardOutput(gpio_num_t pin_id, gpio_mode_t mode);

    bool Read(void) const;
    bool Write(bool state);
    bool Toggle(void);

private:
    const gpio_num_t pin_id;
    bool cached_state;

};
