#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ZigbeeToMainEvent
{
    ZTM_NONE,
    ZTM_OPEN,
    ZTM_CLOSE,
    ZTM_STOP,
    ZTM_NETWORK_JOINED,
    ZTM_NETWORK_LEFT,
};

enum MainToZigbeeEvent
{
    MTZ_NONE,
    MTZ_OPENED,
    MTZ_CLOSED,
    MTZ_MOVING,
    MTZ_UPDATE_TEMPERATURE,
    MTZ_UPDATE_HUMIDITY,
    MTZ_UNKNOWN,
};

typedef struct
{
    MainToZigbeeEvent   event;
    int16_t             payload;
} MainToZigbeePacket;

bool AppZigbee_Init(QueueHandle_t queue_zm, QueueHandle_t queue_mz);

bool AppZigbee_Reset(void);
