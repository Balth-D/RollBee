#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "board/BoardOutput.hpp"
#include "board/BoardInput.hpp"
#include "board/BoardI2cMaster.hpp"
#include "board/BoardTemperature.hpp"

#include "AppZigbee.hpp"

static const char *TAG = "main.cpp";

// --- LOCAL TYPEDEFS ---
enum CoverState {
    STATE_UNKNOWN,
    STATE_MOVING,
    STATE_OPENED,
    STATE_CLOSED,
};

// --- LOCAL VARIABLES ---
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

static TimerHandle_t timer_buttons_hndl = NULL;
static TimerHandle_t timer_sensors_hndl = NULL;
static TimerHandle_t timer_reset_hndl   = NULL;
static volatile BoardInput* buttons_pending_isr = NULL;
static volatile BoardInput* sensors_pending_isr = NULL;

static QueueHandle_t queue_zigbee_to_main = NULL;
static QueueHandle_t queue_main_to_zigbee = NULL;

static volatile CoverState cover_state = STATE_UNKNOWN;

static uint16_t humidity    = 0;
static int16_t temperature  = 0;

// --- LOCAL FUNCTION PROTOTYPES --
static void InputInterruptHandler(void* data);

static void ButtonsHandlerClbk(TimerHandle_t xTimer);
static void SensorsHandlerClbk(TimerHandle_t xTimer);
static void ResetHandlerClbk(TimerHandle_t xTimer);

static void TriggerLoad(void);

// --- LOCAL FUNCTION DEFINITION --
static void InputInterruptHandler(void* data)
{
    BoardInput* in = (BoardInput*) data;

    if (in == &sw_1
    || in == &sw_2
    || in == &sw_3)
    {
        xTimerStartFromISR(timer_buttons_hndl, NULL);
        buttons_pending_isr = in;
    }
    else if (in == &sense_h
        || in == &sense_l)
    {
        xTimerStartFromISR(timer_sensors_hndl, NULL);
        sensors_pending_isr = in;
    }
}

static void ButtonsHandlerClbk(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Push buttons handler");

    if (buttons_pending_isr == &sw_1)
    {
        // Start reset pending timer
        xTimerStartFromISR(timer_reset_hndl, NULL);
    }
    else if (buttons_pending_isr == &sw_2)
    {
        // Trigger the load, sensors will update the Zigbee state automatically
        TriggerLoad();
    }
    else if (buttons_pending_isr == &sw_3)
    {
    }

    buttons_pending_isr = NULL;
}

static void SensorsHandlerClbk(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Sensors switch handler");

    MainToZigbeePacket msg;

    // Check all sensors
    if (sensors_pending_isr == &sense_h)
    {
        if (sense_h.Read())
        {
            ESP_LOGI(TAG, "Cover state opened");
            cover_state = STATE_OPENED;
            msg.event = MTZ_OPENED;
        }
        else
        {
            ESP_LOGI(TAG, "Cover state unknown");
            cover_state = STATE_UNKNOWN;
            msg.event = MTZ_UNKNOWN;
        }
    }
    else if (sensors_pending_isr == &sense_l)
    {
        if (sense_l.Read())
        {
            ESP_LOGI(TAG, "Cover state closed");
            cover_state = STATE_CLOSED;
            msg.event = MTZ_CLOSED;
        }
        else
        {
            ESP_LOGI(TAG, "Cover state unknown");
            cover_state = STATE_UNKNOWN;
            msg.event = MTZ_UNKNOWN;
        }
    }

    xQueueSendFromISR(queue_main_to_zigbee, &msg, NULL);

    sensors_pending_isr = NULL;
}

static void ResetHandlerClbk(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Reset handler");
    
    // Check if button 1 is still pressed after 5s
    if (!sw_1.Read())
    {
        ESP_LOGI(TAG, "Factory reset triggered");
        AppZigbee_Reset();
    }
}

static void TriggerLoad(void)
{
    // Just make a 200ms pulse
    load.Write(true);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    load.Write(false);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Hardware test start");

    // Blink LEDs
    led_1.Write(true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    led_1.Write(false);

    led_2.Write(true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    led_2.Write(false);

    led_3.Write(true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    led_3.Write(false);

    // LED 1 is on until device has joined network
    led_1.Write(true);

    // Create deffered interrupt handler
    timer_buttons_hndl = xTimerCreate("Timer-Buttons", 100  / portTICK_PERIOD_MS, pdFALSE, NULL, ButtonsHandlerClbk);
    timer_sensors_hndl = xTimerCreate("Timer-Sensors", 1000 / portTICK_PERIOD_MS, pdFALSE, NULL, SensorsHandlerClbk);
    timer_reset_hndl   = xTimerCreate("Timer-Reset", 5000 / portTICK_PERIOD_MS, pdFALSE, NULL, ResetHandlerClbk);

    // Create queues for Zigbee events processing
    queue_zigbee_to_main = xQueueCreate(10, sizeof(ZigbeeToMainEvent));
    queue_main_to_zigbee = xQueueCreate(10, sizeof(MainToZigbeePacket));

    // Enable inputs interrupts and link them to the handler
    sw_1.SetInterruptCallback(InputInterruptHandler, &sw_1);
    sw_1.EnableInterrupt(GPIO_INTR_NEGEDGE);
    sw_2.SetInterruptCallback(InputInterruptHandler, &sw_2);
    sw_2.EnableInterrupt(GPIO_INTR_NEGEDGE);
    sw_3.SetInterruptCallback(InputInterruptHandler, &sw_3);
    sw_3.EnableInterrupt(GPIO_INTR_NEGEDGE);

    sense_h.SetInterruptCallback(InputInterruptHandler, &sense_h);
    sense_h.EnableInterrupt(GPIO_INTR_ANYEDGE);

    sense_l.SetInterruptCallback(InputInterruptHandler, &sense_l);
    sense_l.EnableInterrupt(GPIO_INTR_ANYEDGE);

    // Configure I²C and temperature sensor
    i2c_master.Configure(GPIO_NUM_4, GPIO_NUM_5, false, 40000);

    sensor.StartMeasurement();

    AppZigbee_Init(queue_zigbee_to_main, queue_main_to_zigbee);

    ZigbeeToMainEvent ztm_msg;
    MainToZigbeePacket mtz_msg;

    // Send first state of the cover to the Zigbee app
    if (sense_h.Read())
    {
        ESP_LOGI(TAG, "Cover state opened");
        cover_state = STATE_OPENED;
        mtz_msg.event = MTZ_OPENED;
    }
    else if (sense_l.Read())
    {
        ESP_LOGI(TAG, "Cover state closed");
        cover_state = STATE_CLOSED;
        mtz_msg.event = MTZ_CLOSED;
    }
    else
    {
        ESP_LOGI(TAG, "Cover state unknown");
        cover_state = STATE_UNKNOWN;
        mtz_msg.event = MTZ_UNKNOWN;
    }

    xQueueSend(queue_main_to_zigbee, &mtz_msg, 0);
    
    while (1)
    {
        ESP_LOGI(TAG, "Temperature is %.2f °C and humidity is %.2f %%", 
            sensor.GetAveragedTemperature() / 100.0,
            sensor.GetAveragedHumidity() / 100.0);

        // Check if temperature/humidity changed
        if (sensor.GetAveragedTemperature() != temperature)
        {
            temperature = sensor.GetAveragedTemperature();

            MainToZigbeePacket msg = { .event = MTZ_UPDATE_TEMPERATURE, .payload = temperature };
            xQueueSend(queue_main_to_zigbee, &msg, 0);
        }
        if (sensor.GetAveragedHumidity() != humidity)
        {
            humidity = sensor.GetAveragedHumidity();

            MainToZigbeePacket msg = { .event = MTZ_UPDATE_HUMIDITY, .payload = static_cast<int16_t>(humidity & 0x3FFF) };
            xQueueSend(queue_main_to_zigbee, &msg, 0);
        }

        // Start sampling
        sensor.Sample();
        
        // If Zigbee has something to say to the main (block 4s otherwise)
        if (xQueueReceive(queue_zigbee_to_main, &ztm_msg, 4e3 / portTICK_PERIOD_MS) == pdPASS)
        {
            switch (ztm_msg)
            {
            case ZTM_NETWORK_JOINED:
            {
                ESP_LOGI(TAG, "Zigbee network joined");
                led_1.Write(false);
                break;
            }
            case ZTM_NETWORK_LEFT:
            {
                ESP_LOGI(TAG, "Zigbee network left, will reboot");
                led_1.Write(true);
                esp_restart();
                break;
            }
            case ZTM_OPEN:
            {
                if (cover_state != STATE_OPENED)
                {
                    ESP_LOGI(TAG, "Opening");
                    TriggerLoad();
                }
                break;
            }
            case ZTM_CLOSE:
            {
                if (cover_state != STATE_CLOSED)
                {
                    ESP_LOGI(TAG, "Closing");
                    TriggerLoad();
                }
                break;
            }
            case ZTM_STOP:
            {
                if (cover_state != STATE_OPENED
                    && cover_state != STATE_CLOSED)
                {
                    ESP_LOGI(TAG, "Stopping");
                    TriggerLoad();
                }
                break;
            }
            default:
                break;
            }
        }
    }
}
