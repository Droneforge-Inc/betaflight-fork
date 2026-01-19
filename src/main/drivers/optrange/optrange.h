#pragma once

#include "common/time.h"
#include "drivers/io_types.h"

typedef struct optrangeHardwarePins_s {
    ioTag_t triggerTag;
    ioTag_t echoTag;
} optrangeHardwarePins_t;
    
typedef struct optrangeRangefinderData_s {
    uint32_t distValue;
    uint8_t distStrength;
    uint8_t distPrecision;
    uint8_t distStatus;
} optrangeRangefinderData_t;

typedef struct optrangeFlowData_s {
    int16_t velX;
    int16_t velY;
    uint8_t flowQuality;
    uint8_t flowStatus;
} optrangeFlowData_t;

struct optrangeDev_s;
typedef void (*optrangeOpInitFuncPtr)(struct optrangeDev_s * dev);
typedef void (*optrangeOpStartFuncPtr)(struct optrangeDev_s * dev);
typedef optrangeRangefinderData_t (*optrangeOpReadRangefinderFuncPtr)(struct optrangeDev_s * dev);
typedef optrangeFlowData_t (*optrangeOpReadFlowFuncPtr)(struct optrangeDev_s * dev);

typedef struct optrangeDev_s {
    timeMs_t delayMs;
    int16_t maxRangeCm;

    // these are full detection cone angles, maximum tilt is half of this
    int16_t detectionConeDeciDegrees; // detection cone angle as in device spec
    int16_t detectionConeExtendedDeciDegrees; // device spec is conservative, in practice have slightly larger detection cone

    // function pointers
    optrangeOpInitFuncPtr init;
    optrangeOpStartFuncPtr update;
    optrangeOpReadRangefinderFuncPtr readRangefinder;
    optrangeOpReadFlowFuncPtr readFlow;
} optrangeDev_t;
