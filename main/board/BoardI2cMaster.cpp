#include "BoardI2cMaster.hpp"

#include "driver/i2c.h"

BoardI2cMaster::BoardI2cMaster(i2c_port_t port)
    : i2c_port{port}
{
}

bool BoardI2cMaster::Configure(gpio_num_t pin_sda, gpio_num_t pin_scl, bool pull_up_enable, uint32_t clock_speed)
{
    i2c_config_t i2c_conf;
    i2c_conf.mode               = I2C_MODE_MASTER;
    i2c_conf.sda_io_num         = pin_sda;
    i2c_conf.sda_pullup_en      = pull_up_enable;
    i2c_conf.scl_io_num         = pin_scl;
    i2c_conf.scl_pullup_en      = pull_up_enable;
    i2c_conf.master.clk_speed   = clock_speed;
    i2c_conf.clk_flags          = 0;

    return (i2c_param_config(i2c_port, &i2c_conf) == ESP_OK
            && i2c_driver_install(i2c_port, i2c_conf.mode, 0, 0, 0) == ESP_OK);
}

bool BoardI2cMaster::WriteByte(uint8_t address, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Make the command
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    // Send the command, with timeout of 1s
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

bool BoardI2cMaster::ReadByte(uint8_t address, uint8_t& data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Make the command
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    // Send the command, with timeout of 1s
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

bool BoardI2cMaster::WriteBlock(uint8_t address, uint8_t* buffer, uint8_t count)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Make the command
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buffer, count, true);
    i2c_master_stop(cmd);
    // Send the command, with timeout of 1s
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

bool BoardI2cMaster::ReadBlock(uint8_t address, uint8_t* buffer, uint8_t count)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Make the command
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, count - 1, I2C_MASTER_ACK);
    i2c_master_read(cmd, buffer + count - 1, 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    // Send the command, with timeout of 1s
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}
