#pragma once
#include "pg/pg.h"
typedef struct {
    uint16_t hover, accelToThrottle; // normalized throttle * 10000
    uint16_t kp[3], kv[3], ki[3];    // SI gains * 1000
    uint8_t maxTiltDeg;
} df3Config_t;
PG_DECLARE(df3Config_t, df3Config);

// Kept in a separate versioned PG so adding calibration cannot reset DF3 gains.
typedef struct {
    float a0, a1, v0; // legacy CRSF channel units: hover(V) = a0 + a1 * V
    uint8_t enabled;
} df3CalibrationConfig_t;
PG_DECLARE(df3CalibrationConfig_t, df3CalibrationConfig);

// Independent PG preserves existing collective calibration and controller gains.
typedef struct {
    uint16_t rotationScale;  // dimensionless * 1000
    int16_t sensorOffset[3]; // lens from COM, body FRD millimetres
} df3FlowConfig_t;
PG_DECLARE(df3FlowConfig_t, df3FlowConfig);
