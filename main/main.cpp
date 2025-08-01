#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "board/BoardOutput.hpp"
#include "board/BoardInput.hpp"

static const char *TAG = "main.cpp";

static BoardOutput led_1(GPIO_NUM_10, GPIO_MODE_OUTPUT);
static BoardOutput led_2(GPIO_NUM_11, GPIO_MODE_OUTPUT);
static BoardOutput led_3(GPIO_NUM_12, GPIO_MODE_OUTPUT);

static BoardInput  sw_1(GPIO_NUM_13, GPIO_MODE_INPUT);
static BoardInput  sw_2(GPIO_NUM_14, GPIO_MODE_INPUT);
static BoardInput  sw_3(GPIO_NUM_22, GPIO_MODE_INPUT);

static void InputInterruptHandler(void* data)
{
    if (data == &sw_1)
    {
        led_1.Toggle();
    }
    else if (data == &sw_2)
    {
        led_2.Toggle();
    }
    else if (data == &sw_3)
    {
        led_3.Toggle();
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

    while (1)
    {
        vTaskDelay(100);
    }
}
