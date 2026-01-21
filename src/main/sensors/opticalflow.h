#pragma once

#include <stdint.h>

#include "common/time.h"
#include "drivers/optrange/optrange.h"
#include "pg/pg.h"

typedef enum {
  OPTICALFLOW_NONE = 0,
  OPTICALFLOW_MTF02 = 1,
} opticalflowType_e;

typedef struct opticalflowConfig_s {
  uint8_t opticalflow_hardware;
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
} opticalflow_t;

bool opticalflowInit(void);

int16_t opticalflowGetLatestVelX(void);
int16_t opticalflowGetLatestVelY(void);

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
uint8_t opticalflowGetLatestFlowQuality(void);
uint8_t opticalflowGetLatestFlowStatus(void);
#endif

void opticalflowUpdate(void);
bool opticalflowProcess(void);
bool opticalflowIsHealthy(void);
