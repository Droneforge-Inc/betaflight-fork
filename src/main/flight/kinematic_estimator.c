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

#include "platform.h"

#include "flight/kinematic_estimator.h"

#include "sensors/acceleration.h"

#define METERS_PER_SECOND_SQUARED_PER_G 9.80665f

// Flight-level runtime owner for the generated kinematic EKF model.
static kinematicFilter_t kinematicEstimator;

static float kinematicEstimatorAccelToMetersPerSecondSquared(float accelBody)
{
    return accelBody * acc.dev.acc_1G_rec * METERS_PER_SECOND_SQUARED_PER_G;
}

void kinematicEstimatorInit(void)
{
    kinematicFilterInit(&kinematicEstimator);
}

void kinematicEstimatorReset(void)
{
    kinematicFilterReset(&kinematicEstimator);
}

void kinematicEstimatorResetState(const kinematicState_t *state)
{
    if (!state) {
        kinematicEstimatorReset();
        return;
    }

    kinematicFilterResetState(&kinematicEstimator, state);
}

void kinematicEstimatorPredictFromImu(float accelBodyX, float accelBodyY, float accelBodyZ, const quaternion *attitudeQuat, float dt)
{
    if (!attitudeQuat || dt <= 0.0f) {
        return;
    }

    kinematicFilterPredictInputs(&kinematicEstimator,
        kinematicEstimatorAccelToMetersPerSecondSquared(accelBodyX),
        kinematicEstimatorAccelToMetersPerSecondSquared(accelBodyY),
        kinematicEstimatorAccelToMetersPerSecondSquared(accelBodyZ),
        attitudeQuat->w, attitudeQuat->x, attitudeQuat->y, attitudeQuat->z,
        dt);
}

const kinematicFilter_t *kinematicEstimatorGetFilter(void)
{
    return &kinematicEstimator;
}

const kinematicState_t *kinematicEstimatorGetState(void)
{
    return &kinematicEstimator.state;
}
