#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_OPTFLOW_MTF

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/io.h"
#include "drivers/optrange/optrange.h"
#include "drivers/optrange/optrange_mtf.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/opticalflow.h"
#include "sensors/battery.h"

opticalflow_t opticalflow;

#define OPTICALFLOW_HARDWARE_TIMEOUT_MS         500     // Accept 500ms of non-responsive sensor, report HW failure otherwise

PG_REGISTER_WITH_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig, PG_OPTICALFLOW_CONFIG, 0);

PG_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig,
    .opticalflow_hardware = OPTICALFLOW_NONE,
);

static bool opticalflowDetect(optrangeDev_t *dev, uint8_t opticalflowHardwareToUse)
{
    opticalflowType_e opticalflowHardware = OPTICALFLOW_NONE;
    requestedSensors[SENSOR_INDEX_OPTICALFLOW] = opticalflowHardwareToUse;

    switch (opticalflowHardwareToUse) {
    case OPTICALFLOW_MTF02:
#if defined(USE_RANGEFINDER_OPTFLOW_MTF)
        if (mtf02Detect(dev)) {
            opticalflowHardware = OPTICALFLOW_MTF02;
            rescheduleTask(TASK_OPTICALFLOW, TASK_PERIOD_MS(OPTRANGE_MTF_TASK_PERIOD_MS));
        }
#endif
        break;
    case OPTICALFLOW_NONE:
        opticalflowHardware = OPTICALFLOW_NONE;
        break;
    }

    if (opticalflowHardware == OPTICALFLOW_NONE) {
        sensorsClear(SENSOR_OPTICALFLOW);
        return false;
    }

    detectedSensors[SENSOR_INDEX_OPTICALFLOW] = opticalflowHardware;
    sensorsSet(SENSOR_OPTICALFLOW);
    return true;
}

bool opticalflowInit(void)
{
    if (!opticalflowDetect(&opticalflow.dev, opticalflowConfig()->opticalflow_hardware)) {
        return false;
    }

    // opticalflow.dev.init(&opticalflow.dev);
    opticalflow.velX = 0;
    opticalflow.velY = 0;
    opticalflow.flowQuality = 0;
    opticalflow.flowStatus = 0;
    opticalflow.lastValidResponseTimeMs = millis();
    
    return true;
}

static int16_t applyMedianFilter(int16_t newReading, bool isVelX)
{
    #define FLOW_SAMPLES_MEDIAN 5
    static int16_t filterVelX[FLOW_SAMPLES_MEDIAN];
    static int16_t filterVelY[FLOW_SAMPLES_MEDIAN];
    static int filterVelXIndex = 0;
    static int filterVelYIndex = 0;
    static bool medianFilterReadyX = false;
    static bool medianFilterReadyY = false;

    if (isVelX) {
        filterVelX[filterVelXIndex] = newReading;
        ++filterVelXIndex;
        if (filterVelXIndex == FLOW_SAMPLES_MEDIAN) {
            filterVelXIndex = 0;
            medianFilterReadyX = true;
        }
    } else {
        filterVelY[filterVelYIndex] = newReading;
        ++filterVelYIndex;
        if (filterVelYIndex == FLOW_SAMPLES_MEDIAN) {
            filterVelYIndex = 0;
            medianFilterReadyY = true;
        }
    }

    return isVelX && medianFilterReadyX ? quickMedianFilter5((int32_t*)filterVelX) : !isVelX && medianFilterReadyY ? quickMedianFilter5((int32_t*)filterVelY) : newReading;
}

static int16_t applyLowPassFilter(int16_t newReading, bool isVelX)
{
    static float smoothX = 0.0f;
    static float smoothY = 0.0f;

    float alpha = 0.02f;

    if (isVelX) {
        smoothX = alpha * newReading + (1 - alpha) * smoothX;
    } else {
        smoothY = alpha * newReading + (1 - alpha) * smoothY;
    }
    return (int16_t)(isVelX ? smoothX : smoothY);
}

void opticalflowUpdate(void)
{
    if (opticalflow.dev.update) {
        opticalflow.dev.update(&opticalflow.dev);
    }
}

bool opticalflowProcess(void)
{
    if (opticalflow.dev.readFlow) {
        optrangeFlowData_t flowData = opticalflow.dev.readFlow(&opticalflow.dev);
        opticalflow.velX = applyLowPassFilter(applyMedianFilter(flowData.velX, true), true);
        opticalflow.velY = applyLowPassFilter(applyMedianFilter(flowData.velY, false), false);

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
        opticalflow.flowQuality = flowData.flowQuality;
        opticalflow.flowStatus = flowData.flowStatus;
#endif

        opticalflow.lastValidResponseTimeMs = millis();
    } else {
#ifdef USE_RANGEFINDER_OPTFLOW_MTF
        opticalflow.flowStatus = 0;
#endif
    }

    DEBUG_SET(DEBUG_OPTICALFLOW, 1, opticalflow.velX);
    DEBUG_SET(DEBUG_OPTICALFLOW, 2, opticalflow.velY);

    return true;
}

int16_t opticalflowGetLatestVelX(void)
{
    return opticalflow.velX;
}

int16_t opticalflowGetLatestVelY(void) {
    return opticalflow.velY;
}

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
uint8_t opticalflowGetLatestFlowQuality(void)
{
    return opticalflow.flowQuality;
}

uint8_t opticalflowGetLatestFlowStatus(void)
{
    return opticalflow.flowStatus;
}
#endif

bool opticalflowIsHealthy(void)
{
    return (millis() - opticalflow.lastValidResponseTimeMs) < OPTICALFLOW_HARDWARE_TIMEOUT_MS;
}
#endif
