#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "drivers/optrange/optrange.h"
#include "pg/pg.h"

typedef enum {
  OPTICALFLOW_NONE = 0,
  OPTICALFLOW_MTF02 = 1,
} opticalflowType_e;

typedef enum {
  OPTICALFLOW_ALIGN_CW0_DEG = 0,
  OPTICALFLOW_ALIGN_CW90_DEG,
  OPTICALFLOW_ALIGN_CW180_DEG,
  OPTICALFLOW_ALIGN_CW270_DEG,
} opticalflowAlign_e;

typedef struct opticalflowConfig_s {
  uint8_t opticalflow_hardware;
  uint8_t opticalflow_align;
} opticalflowConfig_t;

PG_DECLARE(opticalflowConfig_t, opticalflowConfig);

typedef struct opticalflow_s {
  optrangeDev_t dev;
  int16_t velX; // cm/s @ 1m
  int16_t velY; // cm/s @ 1m
#ifdef USE_RANGEFINDER_OPTFLOW_MTF
  uint8_t flowQuality; // 0-255, higher is better
  uint8_t flowStatus;  // 0 is invalid, 1 is valid
#endif

  timeMs_t lastValidResponseTimeMs;
#ifdef SITL
  uint32_t processedFrameSequence; // Latest UART sequence observed by a callback.
#endif
} opticalflow_t;

typedef struct opticalflowMeasurement_s {
  int16_t velX; // cm/s @ 1m
  int16_t velY; // cm/s @ 1m
  bool isHealthy;
#ifdef USE_RANGEFINDER_OPTFLOW_MTF
  uint8_t flowQuality; // 0-255, higher is better
  uint8_t flowStatus;  // 0 is invalid, 1 is valid
#endif
} opticalflowMeasurement_t;

bool opticalflowInit(void);

void opticalflowGetLatestMeasurement(opticalflowMeasurement_t *measurement);
int16_t opticalflowGetLatestVelX(void);
int16_t opticalflowGetLatestVelY(void);

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
uint8_t opticalflowGetLatestFlowQuality(void);
uint8_t opticalflowGetLatestFlowStatus(void);
#endif

void opticalflowUpdate(void);
bool opticalflowProcess(void);
bool opticalflowIsHealthy(void);

#ifdef SITL
uint32_t opticalflowGetProcessedFrameSequence(void);
#endif
