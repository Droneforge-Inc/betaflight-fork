/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "drivers/rangefinder/rangefinder.h"

#include "pg/pg.h"

typedef enum {
    RANGEFINDER_NONE    = 0,
    RANGEFINDER_HCSR04  = 1,
    RANGEFINDER_TFMINI  = 2,
    RANGEFINDER_TF02    = 3,
    RANGEFINDER_MTF02   = 4,
} rangefinderType_e;

typedef struct rangefinderConfig_s {
    uint8_t rangefinder_hardware;
} rangefinderConfig_t;

PG_DECLARE(rangefinderConfig_t, rangefinderConfig);

typedef struct rangefinder_s {
#ifdef USE_RANGEFINDER_OPTFLOW_MTF
    optrangeDev_t dev;
#else
    rangefinderDev_t dev;
#endif
    float maxTiltCos;
    int32_t rawAltitude;
    int32_t calculatedAltitude;
#ifdef USE_RANGEFINDER_TF
    uint16_t strength;
#endif
#ifdef USE_RANGEFINDER_OPTFLOW_MTF
    uint8_t distStrength; // 0-255
    uint8_t distStrengthPrecision; // 0-255, lower is better
    uint8_t distStatus; // 0 is invalid, 1 is valid
#endif
    timeMs_t lastValidResponseTimeMs;

    bool snrThresholdReached;
    int32_t dynamicDistanceThreshold;
    int16_t snr;
} rangefinder_t;

void rangefinderResetDynamicThreshold(void);
bool rangefinderInit(void);

int32_t rangefinderGetLatestAltitude(void);
int32_t rangefinderGetLatestRawAltitude(void);
#ifdef USE_RANGEFINDER_TF
uint16_t rangefinderGetLatestStrength(void);
#endif
#ifdef USE_RANGEFINDER_OPTFLOW_MTF
uint8_t rangefinderGetLatestDistStrength(void);
uint8_t rangefinderGetLatestDistPrecision(void);
#endif

void rangefinderUpdate(void);
bool rangefinderProcess(float cosTiltAngle);
bool rangefinderIsHealthy(void);
