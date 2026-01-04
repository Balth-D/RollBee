
#include "BoardTemperature.hpp"

#include <math.h>   // pow, log

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// SI7021 device address
inline constexpr uint8_t SI7021_ADDR										= 0x40;

// SI7021 device commands
inline constexpr uint8_t SI7021_RESET_COMMAND								= 0xFE;
inline constexpr uint8_t SI7021_READ_TEMPERATURE_W_HOLD_COMMAND				= 0xE3;
inline constexpr uint8_t SI7021_READ_TEMPERATURE_WO_HOLD_COMMAND			= 0xF3;
inline constexpr uint8_t SI7021_READ_HUMIDITY_W_HOLD_COMMAND				= 0xE5;
inline constexpr uint8_t SI7021_READ_HUMIDITY_WO_HOLD_COMMAND				= 0xF5;
inline constexpr uint16_t SI7021_READ_SERIAL_FIRST_8BYTES_COMMAND			= 0xFA0F;
inline constexpr uint16_t SI7021_READ_SERIAL_LAST_6BYTES_COMMAND			= 0xFCC9;
inline constexpr uint8_t SI7021_WRITE_USER_REG_COMMAND						= 0xE6;
inline constexpr uint8_t SI7021_READ_USER_REG_COMMAND						= 0xE7;

inline constexpr uint16_t RESET_TIME_US										= 15000;

// All these constants have been multiplied by 100 to avoid floats
// Processing constants
inline constexpr int16_t SI7021_TEMPERATURE_COEFFICIENT      				= -15;
inline constexpr float SI7021_TEMPERATURE_COEFFICIENT_FLOAT					= -0.15;
inline constexpr float SI7021_CONSTANT_A									= 8.13;
inline constexpr float SI7021_CONSTANT_B									= 1762.39;
inline constexpr float SI7021_CONSTANT_C									= 235.66;

// Coefficients for temperature computation
inline constexpr int16_t TEMPERATURE_COEFF_MUL								= 17572; // 175.72
inline constexpr int16_t TEMPERATURE_COEFF_ADD								= -4685; // -46.85

// Coefficients for relative humidity computation
inline constexpr int16_t HUMIDITY_COEFF_MUL									= 12500; // 125
inline constexpr int16_t HUMIDITY_COEFF_ADD									= -600; // -6

// Conversion timings in µs
inline constexpr uint32_t SI7021_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b	= 50000;
inline constexpr uint32_t SI7021_TEMPERATURE_CONVERSION_TIME_T_13b_RH_10b	= 25000;
inline constexpr uint32_t SI7021_TEMPERATURE_CONVERSION_TIME_T_12b_RH_8b	= 13000;
inline constexpr uint32_t SI7021_TEMPERATURE_CONVERSION_TIME_T_11b_RH_11b	= 7000;
inline constexpr uint32_t SI7021_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b		= 16000;
inline constexpr uint32_t SI7021_HUMIDITY_CONVERSION_TIME_T_13b_RH_10b		= 5000;
inline constexpr uint32_t SI7021_HUMIDITY_CONVERSION_TIME_T_12b_RH_8b		= 3000;
inline constexpr uint32_t SI7021_HUMIDITY_CONVERSION_TIME_T_11b_RH_11b		= 8000;

// SI7021 User Register masks and bit position
inline constexpr uint8_t SI7021_USER_REG_RESOLUTION_MASK					= 0x81;
inline constexpr uint8_t SI7021_USER_REG_END_OF_BATTERY_MASK				= 0x40;
inline constexpr uint8_t SI7021_USER_REG_ENABLE_ONCHIP_HEATER_MASK			= 0x4;
inline constexpr uint8_t SI7021_USER_REG_DISABLE_OTP_RELOAD_MASK			= 0x2;
inline constexpr uint32_t SI7021_USER_REG_RESERVED_MASK						= (~(SI7021_USER_REG_RESOLUTION_MASK |	SI7021_USER_REG_END_OF_BATTERY_MASK | SI7021_USER_REG_ENABLE_ONCHIP_HEATER_MASK | SI7021_USER_REG_DISABLE_OTP_RELOAD_MASK));

// HTU User Register values
// Resolution
inline constexpr uint8_t SI7021_USER_REG_RESOLUTION_T_14b_RH_12b			= 0x00;
inline constexpr uint8_t SI7021_USER_REG_RESOLUTION_T_13b_RH_10b			= 0x80;
inline constexpr uint8_t SI7021_USER_REG_RESOLUTION_T_12b_RH_8b				= 0x01;
inline constexpr uint8_t SI7021_USER_REG_RESOLUTION_T_11b_RH_11b			= 0x81;

// End of battery status
inline constexpr uint8_t SI7021_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V		= 0x00;
inline constexpr uint8_t SI7021_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V		= 0x40;
// Enable on chip heater
inline constexpr uint8_t SI7021_USER_REG_ONCHIP_HEATER_ENABLE				= 0x04;
inline constexpr uint8_t SI7021_USER_REG_OTP_RELOAD_DISABLE					= 0x02;

BoardTemperature::BoardTemperature(BoardI2cMaster& i2c_master)
    : i2c(i2c_master)
{
}

void BoardTemperature::StartMeasurement(void)
{
    // Reset the sensor
    i2c.WriteByte(SI7021_ADDR, SI7021_RESET_COMMAND);
    vTaskDelay(RESET_TIME_US / (1000 * portTICK_PERIOD_MS));

    // Start the first measurement
    if (DetectSensor())
    {
        sensor_detected = true;
        StartMeasurementInternal(MEASURE_TEMPERATURE);
    }
}

uint16_t BoardTemperature::GetHumidity(void) const
{
    return humidity;
}

uint16_t BoardTemperature::GetAveragedHumidity(void) const
{
    return averaged_humidity;
}

uint16_t BoardTemperature::GetHumidityCompensated(void) const
{
    return humidity + ((2500 - (temperature)) * SI7021_TEMPERATURE_COEFFICIENT);
}

int16_t BoardTemperature::GetTemperature(void) const
{
    return temperature;
}

int16_t BoardTemperature::GetAveragedTemperature(void) const
{
    return averaged_temperature;
}

int16_t BoardTemperature::GetDewPoint(void) const
{
    return dew_point;
}

bool BoardTemperature::DetectSensor(void)
{
    uint8_t data1[8] = {0};
    uint8_t data2[6] = {0};
    uint8_t cmd[2];
    
    cmd[0] = (SI7021_READ_SERIAL_FIRST_8BYTES_COMMAND >> 8) & 0xFF;
	cmd[1] = SI7021_READ_SERIAL_FIRST_8BYTES_COMMAND & 0xFF;

    bool status = i2c.WriteBlock(SI7021_ADDR, cmd, 2);
    
    if (status)
    {
        status = i2c.ReadBlock(SI7021_ADDR, (uint8_t*) data1, 8);
    }

    cmd[0] = (SI7021_READ_SERIAL_LAST_6BYTES_COMMAND >> 8) & 0xFF;
    cmd[1] = SI7021_READ_SERIAL_LAST_6BYTES_COMMAND & 0xFF;
    
    if (status)
    {
        status = i2c.WriteBlock(SI7021_ADDR, cmd, 2);
    }

    if (status)
    {
        status = i2c.ReadBlock(SI7021_ADDR, (uint8_t *) data2, 6);
    }

    // Si7021 or Si7020 detected
    if (status && 
        (data2[0] == 0x15 || data2[0] == 0x14))
    {
        return true;
    }

    return false;
}

bool BoardTemperature::Sample(void)
{
    if (!sensor_detected)
    {
        return false;
    }

    uint8_t adc_raw[2];
    // Read incoming data
    bool status = i2c.ReadBlock(SI7021_ADDR, adc_raw, 2);
    int32_t val = ((adc_raw[0] << 8) | adc_raw[1]) & 0xFFFC; // Receive MSB first

    if (status)
    {
        if (measurement == MEASURE_TEMPERATURE)
        {
            val = (val * TEMPERATURE_COEFF_MUL) / 65536 + TEMPERATURE_COEFF_ADD;
            temperature = val;
            averaged_temperature = (AVG_BETA * averaged_temperature) + (1 - AVG_BETA) * temperature;
            
            // Start next measurement. Will be finished before next task entry
            StartMeasurementInternal(MEASURE_HUMIDITY);
        }
        else
        {
            val = (val * HUMIDITY_COEFF_MUL) / 65536 + HUMIDITY_COEFF_ADD;
            humidity = static_cast<uint16_t>(val);
            averaged_humidity = (AVG_BETA * averaged_humidity) + (1 - AVG_BETA) * humidity;

            // Compute dew point
            //float partial_pressure = pow(10, SI7021_CONSTANT_A - SI7021_CONSTANT_B / ((static_cast<float>(temperature) / 100) + SI7021_CONSTANT_C));
            //partial_pressure = - SI7021_CONSTANT_B / ((log10((static_cast<float>(humidity) / 100) * partial_pressure / 100)) - SI7021_CONSTANT_A) - SI7021_CONSTANT_C;
            //dew_point = static_cast<int16_t>(partial_pressure * 100);
            
            // Start next measurement. Will be finished before next task entry
            StartMeasurementInternal(MEASURE_TEMPERATURE);
        }
    }
    else
    {
        // Sensor was already busy reading environmental value
        return false;
    }

    return true;
}


bool BoardTemperature::StartMeasurementInternal(MeasType next_mes)
{
    bool status = false;

    if (next_mes == MeasType::MEASURE_HUMIDITY)
        status = i2c.WriteByte(SI7021_ADDR, SI7021_READ_HUMIDITY_WO_HOLD_COMMAND);
    else if (next_mes == MeasType::MEASURE_TEMPERATURE)
        status = i2c.WriteByte(SI7021_ADDR, SI7021_READ_TEMPERATURE_WO_HOLD_COMMAND);

    if (status)
    {
        measurement = next_mes;
    }

    return status;
}
