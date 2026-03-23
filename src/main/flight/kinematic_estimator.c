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
#define METERS_PER_CENTIMETER 0.01f
#define METERS_PER_CENTIMETER_SQUARED (METERS_PER_CENTIMETER * METERS_PER_CENTIMETER)
#define RANGEFINDER_VARIANCE_SCALE_MAX 20.0f
#define OPTICALFLOW_VARIANCE_SCALE_MAX 20.0f
#define RANGEFINDER_STRENGTH_GAIN 4.0f
#define DISTANCE_VARIANCE_GAIN 2.5f
#define OPTICALFLOW_QUALITY_GAIN 4.0f

// Flight-level runtime owner for the generated kinematic EKF model.
static kinematicFilter_t kinematicEstimator;

static float kinematicEstimatorAccelToMetersPerSecondSquared(float accelBody)
{
    return accelBody * acc.dev.acc_1G_rec * METERS_PER_SECOND_SQUARED_PER_G;
}

static float kinematicEstimatorCentimetersToMeters(float valueCm)
{
    return valueCm * METERS_PER_CENTIMETER;
}

static float kinematicEstimatorOpticalflowToMetersPerSecond(float flowVelocityCmPerSecondAtOneMeter, float altitudeCm)
{
    return flowVelocityCmPerSecondAtOneMeter * altitudeCm * METERS_PER_CENTIMETER_SQUARED;
}

static float kinematicEstimatorGetDistanceVarianceScale(int32_t distanceCm)
{
    if (distanceCm > 0 && rangefinderMaxRangeCm > 0) {
        const float distanceNorm = constrainf(distanceCm / (float)rangefinderMaxRangeCm, 0.0f, 1.0f);

        return 1.0f + DISTANCE_VARIANCE_GAIN * distanceNorm * distanceNorm;
    }

    return 1.0f;
}

static float kinematicEstimatorGetRangefinderVarianceScale(const rangefinderMeasurement_t *rangefinderMeasurement)
{
    float strengthScale = 1.0f;

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
    const float strengthNorm = constrainf(rangefinderMeasurement->distStrength / 255.0f, 0.0f, 1.0f);

    strengthScale = 1.0f + RANGEFINDER_STRENGTH_GAIN * (1.0f - strengthNorm);
#endif

    return constrainf(strengthScale * kinematicEstimatorGetDistanceVarianceScale(rangefinderMeasurement->rawAltitudeCm), 1.0f, RANGEFINDER_VARIANCE_SCALE_MAX);
}

static float kinematicEstimatorGetOpticalflowVarianceScale(const opticalflowMeasurement_t *opticalflowMeasurement, const rangefinderMeasurement_t *rangefinderMeasurement)
{
    float qualityScale = 1.0f;

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
    const float qualityNorm = constrainf(opticalflowMeasurement->flowQuality / 255.0f, 0.0f, 1.0f);
    const float qualityError = 1.0f - qualityNorm;

    qualityScale = 1.0f + OPTICALFLOW_QUALITY_GAIN * qualityError * qualityError;
#else
    UNUSED(opticalflowMeasurement);
#endif

    return constrainf(qualityScale * kinematicEstimatorGetDistanceVarianceScale(rangefinderMeasurement->rawAltitudeCm), 1.0f, OPTICALFLOW_VARIANCE_SCALE_MAX);
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

void kinematicEstimatorPredictFromImu(float accelBodyX, float accelBodyY, float accelBodyZ, const quaternion_t *attitudeQuat, float dt)
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

void kinematicEstimatorUpdateFromOpticalflow(const opticalflowMeasurement_t *opticalflowMeasurement, const rangefinderMeasurement_t *rangefinderMeasurement, const quaternion_t *attitudeQuat)
{
    if (!opticalflowMeasurement || !rangefinderMeasurement || !attitudeQuat) {
        return;
    }

    if (!opticalflowMeasurement->isHealthy || !rangefinderMeasurement->isHealthy || rangefinderMeasurement->calculatedAltitudeCm < 0) {
        return;
    }

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
    if (opticalflowMeasurement->flowStatus == 0 || rangefinderMeasurement->distStatus == 0) {
        return;
    }
#endif

    const float altitudeCm = (float)rangefinderMeasurement->calculatedAltitudeCm;
    const float opticalflowVarianceScale = kinematicEstimatorGetOpticalflowVarianceScale(opticalflowMeasurement, rangefinderMeasurement);

    kinematicFilterUpdateFlowVelocity(&kinematicEstimator,
        kinematicEstimatorOpticalflowToMetersPerSecond((float)opticalflowMeasurement->velX, altitudeCm),
        kinematicEstimatorOpticalflowToMetersPerSecond((float)opticalflowMeasurement->velY, altitudeCm),
        attitudeQuat,
        opticalflowVarianceScale);
}

void kinematicEstimatorUpdateFromRangefinder(const rangefinderMeasurement_t *rangefinderMeasurement)
{
    if (!rangefinderMeasurement || !rangefinderMeasurement->isHealthy || rangefinderMeasurement->calculatedAltitudeCm < 0) {
        return;
    }

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
    if (rangefinderMeasurement->distStatus == 0) {
        return;
    }
#endif

    const float rangefinderVarianceScale = kinematicEstimatorGetRangefinderVarianceScale(rangefinderMeasurement);

    kinematicFilterUpdatePositionZ(&kinematicEstimator,
        kinematicEstimatorCentimetersToMeters((float)rangefinderMeasurement->calculatedAltitudeCm),
        rangefinderVarianceScale);
}

const kinematicFilter_t *kinematicEstimatorGetFilter(void)
{
    return &kinematicEstimator;
}

const kinematicState_t *kinematicEstimatorGetState(void)
{
    return &kinematicEstimator.state;
}
