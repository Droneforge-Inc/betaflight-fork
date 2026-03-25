#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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

#include "sensors/battery.h"
#include "sensors/opticalflow.h"
#include "sensors/sensors.h"

opticalflow_t opticalflow;

#define OPTICALFLOW_HARDWARE_TIMEOUT_MS                                        \
  500 // Accept 500ms of non-responsive sensor, report HW failure otherwise

PG_REGISTER_WITH_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig,
                                PG_OPTICALFLOW_CONFIG, 0);

PG_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig,
                  .opticalflow_hardware = OPTICALFLOW_NONE,
                  .opticalflow_align = OPTICALFLOW_ALIGN_CW0_DEG, );

static void opticalflowAlign(int16_t *velX, int16_t *velY, uint8_t rotation) {
  const int16_t x = *velX;
  const int16_t y = *velY;

  switch (rotation) {
  default:
  case OPTICALFLOW_ALIGN_CW0_DEG:
    break;
  case OPTICALFLOW_ALIGN_CW90_DEG:
    *velX = y;
    *velY = -x;
    break;
  case OPTICALFLOW_ALIGN_CW180_DEG:
    *velX = -x;
    *velY = -y;
    break;
  case OPTICALFLOW_ALIGN_CW270_DEG:
    *velX = -y;
    *velY = x;
    break;
  }
}

static void opticalflowMapRawToFlightFrame(int16_t *velX, int16_t *velY,
                                           uint8_t hardware) {
  UNUSED(velX);

  switch (hardware) {
  case OPTICALFLOW_MTF02:
    *velY = -*velY;
    break;
  default:
    break;
  }
}

static bool opticalflowDetect(optrangeDev_t *dev,
                              uint8_t opticalflowHardwareToUse) {
  opticalflowType_e opticalflowHardware = OPTICALFLOW_NONE;
  requestedSensors[SENSOR_INDEX_OPTICALFLOW] = opticalflowHardwareToUse;

  switch (opticalflowHardwareToUse) {
  case OPTICALFLOW_MTF02:
#if defined(USE_RANGEFINDER_OPTFLOW_MTF)
    if (mtf02Detect(dev)) {
      opticalflowHardware = OPTICALFLOW_MTF02;
      rescheduleTask(TASK_OPTICALFLOW,
                     TASK_PERIOD_MS(OPTRANGE_MTF_TASK_PERIOD_MS));
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

bool opticalflowInit(void) {
  if (!opticalflowDetect(&opticalflow.dev,
                         opticalflowConfig()->opticalflow_hardware)) {
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

void opticalflowGetLatestMeasurement(opticalflowMeasurement_t *measurement) {
  if (!measurement) {
    return;
  }

  measurement->velX = opticalflow.velX;
  measurement->velY = opticalflow.velY;
  measurement->isHealthy = opticalflowIsHealthy();
#ifdef USE_RANGEFINDER_OPTFLOW_MTF
  measurement->flowQuality = opticalflow.flowQuality;
  measurement->flowStatus = opticalflow.flowStatus;
#endif
}

void opticalflowUpdate(void) {
  if (opticalflow.dev.update) {
    opticalflow.dev.update(&opticalflow.dev);
  }
}

bool opticalflowProcess(void) {
  bool hasMeasurement = false;

  if (opticalflow.dev.readFlow) {
    optrangeFlowData_t flowData = opticalflow.dev.readFlow(&opticalflow.dev);
    int16_t velX = flowData.velX;
    int16_t velY = flowData.velY;

    opticalflowMapRawToFlightFrame(&velX, &velY,
                                   opticalflowConfig()->opticalflow_hardware);
    opticalflowAlign(&velX, &velY, opticalflowConfig()->opticalflow_align);
    opticalflow.velX = velX;
    opticalflow.velY = velY;

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
    opticalflow.flowQuality = flowData.flowQuality;
    opticalflow.flowStatus = flowData.flowStatus;
#endif

    opticalflow.lastValidResponseTimeMs = millis();
    hasMeasurement = true;
  } else {
#ifdef USE_RANGEFINDER_OPTFLOW_MTF
    opticalflow.flowStatus = 0;
#endif
  }

  DEBUG_SET(DEBUG_OPTICALFLOW, 1, opticalflow.velX);
  DEBUG_SET(DEBUG_OPTICALFLOW, 2, opticalflow.velY);
  return hasMeasurement;
}

int16_t opticalflowGetLatestVelX(void) { return opticalflow.velX; }

int16_t opticalflowGetLatestVelY(void) { return opticalflow.velY; }

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
uint8_t opticalflowGetLatestFlowQuality(void) {
  return opticalflow.flowQuality;
}

uint8_t opticalflowGetLatestFlowStatus(void) { return opticalflow.flowStatus; }
#endif

bool opticalflowIsHealthy(void) {
  return (millis() - opticalflow.lastValidResponseTimeMs) <
         OPTICALFLOW_HARDWARE_TIMEOUT_MS;
}
#endif
