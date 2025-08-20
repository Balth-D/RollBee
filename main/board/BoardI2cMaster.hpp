#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "hal/gpio_types.h"
#include "hal/i2c_types.h"

class BoardI2cMaster
{
public:
    BoardI2cMaster(i2c_port_t port);

    bool Configure(gpio_num_t pin_sda, gpio_num_t pin_scl, bool pull_up_enable, uint32_t clock_speed);

    bool WriteByte(uint8_t address, uint8_t data);
    bool ReadByte(uint8_t address, uint8_t& data);

    bool WriteBlock(uint8_t address, uint8_t* buffer, uint8_t count);
    bool ReadBlock(uint8_t address, uint8_t* buffer, uint8_t count);

private:
    i2c_port_t i2c_port;

};
