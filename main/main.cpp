#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "board/BoardOutput.hpp"
#include "board/BoardInput.hpp"
#include "board/BoardI2cMaster.hpp"
#include "board/BoardTemperature.hpp"

static const char *TAG = "main.cpp";

static BoardOutput led_1(GPIO_NUM_10, GPIO_MODE_OUTPUT);
static BoardOutput led_2(GPIO_NUM_11, GPIO_MODE_OUTPUT);
static BoardOutput led_3(GPIO_NUM_12, GPIO_MODE_OUTPUT);

static BoardOutput load(GPIO_NUM_26, GPIO_MODE_OUTPUT);

static BoardInput  sw_1(GPIO_NUM_13, GPIO_MODE_INPUT);
static BoardInput  sw_2(GPIO_NUM_14, GPIO_MODE_INPUT);
static BoardInput  sw_3(GPIO_NUM_22, GPIO_MODE_INPUT);

static BoardInput  sense_h(GPIO_NUM_27, GPIO_MODE_INPUT);
static BoardInput  sense_l(GPIO_NUM_3, GPIO_MODE_INPUT);

static BoardI2cMaster i2c_master(I2C_NUM_0);
static BoardTemperature sensor(i2c_master);

static void InputInterruptHandler(void* data)
{
    if (data == &sw_1)
    {
        led_1.Toggle();
        load.Toggle();
    }
    else if (data == &sw_2)
    {
        led_2.Toggle();
    }
    else if (data == &sw_3)
    {
        led_3.Toggle();
    }
    else if (data == &sense_h)
    {
        led_2.Write(sense_h.Read());
    }
    else if (data == &sense_l)
    {
        led_3.Write(sense_l.Read());
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Hardware test start");

    led_1.Write(true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    led_1.Write(false);

    led_2.Write(true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    led_2.Write(false);

    led_3.Write(true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    led_3.Write(false);

    sw_1.SetInterruptCallback(InputInterruptHandler, &sw_1);
    sw_1.EnableInterrupt(GPIO_INTR_NEGEDGE);
    sw_2.SetInterruptCallback(InputInterruptHandler, &sw_2);
    sw_2.EnableInterrupt(GPIO_INTR_NEGEDGE);
    sw_3.SetInterruptCallback(InputInterruptHandler, &sw_3);
    sw_3.EnableInterrupt(GPIO_INTR_NEGEDGE);

    sense_h.SetInterruptCallback(InputInterruptHandler, &sense_h);
    sense_h.EnableInterrupt(GPIO_INTR_POSEDGE);

    sense_l.SetInterruptCallback(InputInterruptHandler, &sense_l);
    sense_l.EnableInterrupt(GPIO_INTR_POSEDGE);

    i2c_master.Configure(GPIO_NUM_4, GPIO_NUM_5, false, 40000);

    sensor.StartMeasurement();
    
    while (1)
    {
        ESP_LOGI(TAG, "Temperature is %.2f °C and humidity is %.2f %%", 
            sensor.GetTemperature() / 100.0,
            sensor.GetHumidityCompensated() / 100.0);
            
        sensor.Sample();
        vTaskDelay(4e3 / portTICK_PERIOD_MS);
    }
}
