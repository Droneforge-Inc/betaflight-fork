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

#ifdef __cplusplus
extern "C" {
#endif

#include "ekf/kinematic_filter.h"
#include "common/quaternion.h"
#ifdef USE_EKF_GPS
#include "io/gps.h"
#endif
#ifdef USE_EKF_BARO
#include "drivers/time.h"
#endif
#include "sensors/opticalflow.h"
#include "sensors/rangefinder.h"

void kinematicEstimatorInit(void);
void kinematicEstimatorReset(void);
void kinematicEstimatorResetState(const kinematicState_t *state);
void kinematicEstimatorOnArm(void);
void kinematicEstimatorPredictFromImu(float accelBodyX, float accelBodyY, float accelBodyZ, const quaternion_t *attitudeQuat, float dt);
#ifdef USE_EKF_BARO
void kinematicEstimatorInvalidateBaro(void);
void kinematicEstimatorUpdateFromBaro(float baroAltitudeCm, timeUs_t currentTimeUs);
#endif
void kinematicEstimatorUpdateFromOpticalflow(const opticalflowMeasurement_t *opticalflowMeasurement, const rangefinderMeasurement_t *rangefinderMeasurement, const quaternion_t *attitudeQuat);
void kinematicEstimatorUpdateFromRangefinder(const rangefinderMeasurement_t *rangefinderMeasurement);
#ifdef USE_EKF_GPS
void kinematicEstimatorUpdateFromGps(const gpsSolutionData_t *gpsSolution);
#endif

const kinematicFilter_t *kinematicEstimatorGetFilter(void);
const kinematicState_t *kinematicEstimatorGetState(void);

#ifdef __cplusplus
}
#endif
