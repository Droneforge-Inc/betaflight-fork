#pragma once

#include <stdint.h>

#include "drivers/opticalflow/opticalflow.h"
#include "common/time.h"
#include "pg/pg.h"

typedef enum {
    OPTICALFLOW_NONE    = 0,
    OPTICALFLOW_MTF02   = 1,
} opticalflowType_e;

typedef struct opticalflowConfig_s {
    uint8_t opticalflow_hardware;
} opticalflowConfig_t;

PG_DECLARE(opticalflowConfig_t, opticalflowConfig);

typedef struct opticalflow_s {
    // opticalflowDev_t dev;
#ifdef USE_RANGEFINDER_OPTFLOW_MTF
    int16_t velX; // cm/s @ 1m
    int16_t velY; // cm/s @ 1m
    uint8_t flowQuality; // 0-255, higher is better
    uint8_t flowStatus; // 0 is invalid, 1 is valid
#endif
    timeMs_t lastValidResponseTimeMs;

} opticalflow_t;
