#pragma once

#include "BoardI2cMaster.hpp"

#include <stdint.h>
#include <stdbool.h>

class BoardTemperature
{
public:
    BoardTemperature(BoardI2cMaster& i2c_master);

    /// @brief Start a measurement of the humidity
    void StartMeasurement(void);

    /// @brief      Get the current measured humidity
    /// @param      None
    /// @return     The humidity in percentage * 100
    uint16_t GetHumidity(void) const;

    /// @brief      Get the current compensated humidity
    /// @param      None
    /// @return     The humidity in percentage * 100
    uint16_t GetHumidityCompensated(void) const;

    /// @brief      Get the humidity with Exponentially Weighted Average 
    /// @param      None
    /// @return     The humidity in percentage * 100
    uint16_t GetAveragedHumidity(void) const;

    /// @brief      Get the current measured temperature
    /// @param      None
    /// @return     The temperature in degrees Celcius * 100
    int16_t GetTemperature(void) const;

    /// @brief      Get the temperature with Exponentially Weighted Average 
    /// @param      None
    /// @return     The temperature in degrees Celcius * 100
    int16_t GetAveragedTemperature(void) const;

    /// @brief      Get the current dew point (temperature the air needs to be cooled to (at constant pressure) in order to achieve a relative humidity of 100%)
    /// @param      None
    /// @return     The dew point degrees Celcius * 100
    int16_t GetDewPoint(void) const;

    /// @brief  Detect if a sensor is detected on the corresponding I²C line
    /// @param  none
    /// @return True if a HTU21D is detected
    bool DetectSensor(void);

    /// @brief  Read a sample from the sensor
    /// @param  none
    /// @return True in case of success, false otherwise
    bool Sample(void);

private:
    BoardI2cMaster& i2c;

    enum MeasType { MEASURE_TEMPERATURE, MEASURE_HUMIDITY };

    uint16_t humidity   = 0;
    int16_t temperature = 0;
    uint16_t dew_point  = 0;
    MeasType measurement{MEASURE_TEMPERATURE};
    bool sensor_detected{false};

    float AVG_BETA = 0.9;
    int16_t averaged_temperature = 2000;
    uint16_t averaged_humidity   = 5000;

    bool StartMeasurementInternal(MeasType next_mes);
};
