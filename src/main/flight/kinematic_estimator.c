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

#include <math.h>

#include "flight/kinematic_estimator.h"

#include "common/maths.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "sensors/acceleration.h"
#include "sensors/sensors.h"

#define METERS_PER_SECOND_SQUARED_PER_G 9.80665f
#define METERS_PER_CENTIMETER 0.01f
#define METERS_PER_CENTIMETER_SQUARED                                          \
  (METERS_PER_CENTIMETER * METERS_PER_CENTIMETER)
#define GPS_EARTH_RADIUS_M 6378137.0f
#define RANGEFINDER_VARIANCE_SCALE_MAX 20.0f
#define OPTICALFLOW_VARIANCE_SCALE_MAX 20.0f
#define GPS_POSITION_VARIANCE_SCALE_MAX 20.0f
#define GPS_ALTITUDE_VARIANCE_SCALE_MAX 20.0f
#define GPS_VELOCITY_VARIANCE_SCALE_MAX 20.0f
#define BARO_VARIANCE_SCALE_MAX 100.0f
#define RANGEFINDER_STRENGTH_GAIN 4.0f
#define DISTANCE_VARIANCE_GAIN 2.5f
#define OPTICALFLOW_QUALITY_GAIN 4.0f
#define BARO_INNOVATION_SCALE_METERS 1.0f
#define BARO_INNOVATION_SCALE_GAIN 1.0f
#define BARO_ROUGHNESS_REFERENCE_MPS2 3.0f
#define BARO_ROUGHNESS_SCALE_GAIN 4.0f
#define BARO_ROUGHNESS_LPF_TIME_CONSTANT_S 0.5f
#define GPS_POSITION_SIGMA_DEFAULT_M 3.0f
#define GPS_ALTITUDE_SIGMA_DEFAULT_M 6.0f
#define GPS_VELOCITY_SIGMA_DEFAULT_MPS 1.0f
#define GPS_POSITION_SIGMA_MIN_M 1.5f
#define GPS_ALTITUDE_SIGMA_MIN_M 3.0f
#define GPS_VELOCITY_SIGMA_MIN_MPS 0.5f
#define GPS_COURSE_MIN_SPEED_MPS 0.5f
#define KINEMATIC_RESET_STATE_VARIANCE 1.0f

// Flight-level runtime owner for the generated kinematic EKF model.
static kinematicFilter_t kinematicEstimator;

#ifdef USE_EKF_BARO
typedef struct kinematicEstimatorBaroState_s {
  bool hasHistory;
  bool needsBiasReset;
  float lastAltitudeMeters;
  float lastRateMetersPerSecond;
  float roughnessMetersPerSecondSquared;
  timeUs_t lastUpdateUs;
} kinematicEstimatorBaroState_t;

static kinematicEstimatorBaroState_t kinematicEstimatorBaroState;
#endif

#ifdef USE_EKF_GPS
typedef struct kinematicEstimatorGpsOrigin_s {
  bool isInitialized;
  int32_t lat;
  int32_t lon;
  float altMeters;
  float cosLat0;
} kinematicEstimatorGpsOrigin_t;

static kinematicEstimatorGpsOrigin_t kinematicEstimatorGpsOrigin;
#endif

static float kinematicEstimatorAccelToMetersPerSecondSquared(float accelBody) {
  return accelBody * acc.dev.acc_1G_rec * METERS_PER_SECOND_SQUARED_PER_G;
}

static float kinematicEstimatorCentimetersToMeters(float valueCm) {
  return valueCm * METERS_PER_CENTIMETER;
}

#ifdef USE_EKF_BARO
static void kinematicEstimatorResetBaroHistory(void) {
  kinematicEstimatorBaroState.hasHistory = false;
  kinematicEstimatorBaroState.needsBiasReset = false;
  kinematicEstimatorBaroState.lastAltitudeMeters = 0.0f;
  kinematicEstimatorBaroState.lastRateMetersPerSecond = 0.0f;
  kinematicEstimatorBaroState.roughnessMetersPerSecondSquared = 0.0f;
  kinematicEstimatorBaroState.lastUpdateUs = 0;
}

void kinematicEstimatorInvalidateBaro(void) {
  kinematicEstimatorResetBaroHistory();
  kinematicEstimatorBaroState.needsBiasReset = true;
}
#endif

#ifdef USE_EKF_GPS
static float kinematicEstimatorMillimetersToMeters(float valueMm) {
  return valueMm * 0.001f;
}
#endif

static float kinematicEstimatorOpticalflowToMetersPerSecond(
    float flowVelocityCmPerSecondAtOneMeter, float altitudeCm) {
  return flowVelocityCmPerSecondAtOneMeter * altitudeCm *
         METERS_PER_CENTIMETER_SQUARED;
}

#ifdef USE_EKF_GPS
static float kinematicEstimatorVarianceScaleFromSigma(float sigma,
                                                      float defaultSigma,
                                                      float maxVarianceScale) {
  if (sigma <= 0.0f || defaultSigma <= 0.0f) {
    return 1.0f;
  }

  return constrainf(sq(sigma / defaultSigma), 1.0e-3f, maxVarianceScale);
}
#endif

static void kinematicEstimatorResetPositionVelocityState(void) {
  kinematicState_t state = kinematicEstimator.state;

  state.posX = 0.0f;
  state.posY = 0.0f;
  state.posZ = 0.0f;
  state.velX = 0.0f;
  state.velY = 0.0f;
  state.velZ = 0.0f;

  kinematicFilterResetState(&kinematicEstimator, &state);

  for (int stateIndex = KINEMATIC_STATE_POS_X;
       stateIndex <= KINEMATIC_STATE_VEL_Z; stateIndex++) {
    for (int column = 0; column < KINEMATIC_ERROR_DIM; column++) {
      kinematicEstimator.P[stateIndex * KINEMATIC_ERROR_DIM + column] = 0.0f;
      kinematicEstimator.P[column * KINEMATIC_ERROR_DIM + stateIndex] = 0.0f;
    }

    kinematicEstimator.P[stateIndex * KINEMATIC_ERROR_DIM + stateIndex] =
        KINEMATIC_RESET_STATE_VARIANCE;
  }
}

static bool kinematicEstimatorResetOnArmEnabled(void) {
#ifdef USE_EKF
  if (isModeActivationConditionPresent(BOXEKFRESET)) {
    return IS_RC_MODE_ACTIVE(BOXEKFRESET);
  }
#endif

  return armingConfig()->reset_kinematic_state_on_arm;
}

#ifdef USE_EKF_GPS
static void kinematicEstimatorResetGpsOrigin(void) {
  kinematicEstimatorGpsOrigin.isInitialized = false;
  kinematicEstimatorGpsOrigin.lat = 0;
  kinematicEstimatorGpsOrigin.lon = 0;
  kinematicEstimatorGpsOrigin.altMeters = 0.0f;
  kinematicEstimatorGpsOrigin.cosLat0 = 1.0f;
}

static void
kinematicEstimatorInitializeGpsOrigin(const gpsSolutionData_t *gpsSolution) {
  const float lat0Rad =
      DEGREES_TO_RADIANS((float)gpsSolution->llh.lat * 1.0e-7f);

  kinematicEstimatorGpsOrigin.isInitialized = true;
  kinematicEstimatorGpsOrigin.lat = gpsSolution->llh.lat;
  kinematicEstimatorGpsOrigin.lon = gpsSolution->llh.lon;
  kinematicEstimatorGpsOrigin.altMeters =
      kinematicEstimatorCentimetersToMeters((float)gpsSolution->llh.altCm);
  kinematicEstimatorGpsOrigin.cosLat0 = cos_approx(lat0Rad);
}

static float kinematicEstimatorGetGpsDopVarianceScale(uint16_t dop,
                                                      uint8_t numSat) {
  float varianceScale = 1.0f;

  if (dop > 0) {
    const float dopValue = 0.01f * dop;
    varianceScale *=
        constrainf(dopValue * dopValue, 1.0f, GPS_POSITION_VARIANCE_SCALE_MAX);
  }

  if (numSat > 0 && numSat < 8) {
    varianceScale *= 1.0f + 0.5f * (8 - numSat);
  }

  return constrainf(varianceScale, 1.0f, GPS_POSITION_VARIANCE_SCALE_MAX);
}

static float kinematicEstimatorGetGpsPositionVarianceScale(
    const gpsSolutionData_t *gpsSolution) {
  if (gpsSolution->acc.hAcc > 0) {
    const float sigmaMeters =
        MAX(kinematicEstimatorMillimetersToMeters((float)gpsSolution->acc.hAcc),
            GPS_POSITION_SIGMA_MIN_M);

    return kinematicEstimatorVarianceScaleFromSigma(
        sigmaMeters, GPS_POSITION_SIGMA_DEFAULT_M,
        GPS_POSITION_VARIANCE_SCALE_MAX);
  }

  return kinematicEstimatorGetGpsDopVarianceScale(
      gpsSolution->dop.hdop ? gpsSolution->dop.hdop : gpsSolution->dop.pdop,
      gpsSolution->numSat);
}

static float kinematicEstimatorGetGpsAltitudeVarianceScale(
    const gpsSolutionData_t *gpsSolution) {
  if (gpsSolution->acc.vAcc > 0) {
    const float sigmaMeters =
        MAX(kinematicEstimatorMillimetersToMeters((float)gpsSolution->acc.vAcc),
            GPS_ALTITUDE_SIGMA_MIN_M);

    return kinematicEstimatorVarianceScaleFromSigma(
        sigmaMeters, GPS_ALTITUDE_SIGMA_DEFAULT_M,
        GPS_ALTITUDE_VARIANCE_SCALE_MAX);
  }

  return kinematicEstimatorGetGpsDopVarianceScale(
      gpsSolution->dop.vdop ? gpsSolution->dop.vdop : gpsSolution->dop.pdop,
      gpsSolution->numSat);
}

static float kinematicEstimatorGetGpsVelocityVarianceScale(
    const gpsSolutionData_t *gpsSolution) {
  if (gpsSolution->acc.sAcc > 0) {
    const float sigmaMetersPerSecond =
        MAX(kinematicEstimatorMillimetersToMeters((float)gpsSolution->acc.sAcc),
            GPS_VELOCITY_SIGMA_MIN_MPS);

    return kinematicEstimatorVarianceScaleFromSigma(
        sigmaMetersPerSecond, GPS_VELOCITY_SIGMA_DEFAULT_MPS,
        GPS_VELOCITY_VARIANCE_SCALE_MAX);
  }

  return kinematicEstimatorGetGpsDopVarianceScale(
      gpsSolution->dop.hdop ? gpsSolution->dop.hdop : gpsSolution->dop.pdop,
      gpsSolution->numSat);
}

static bool
kinematicEstimatorGpsToLocalPosition(const gpsSolutionData_t *gpsSolution,
                                     float *posX, float *posY) {
  const float dLatRad = DEGREES_TO_RADIANS(
      ((float)gpsSolution->llh.lat - (float)kinematicEstimatorGpsOrigin.lat) *
      1.0e-7f);
  const float dLonRad = DEGREES_TO_RADIANS(
      ((float)gpsSolution->llh.lon - (float)kinematicEstimatorGpsOrigin.lon) *
      1.0e-7f);
  const float northMeters = GPS_EARTH_RADIUS_M * dLatRad;
  const float eastMeters =
      GPS_EARTH_RADIUS_M * kinematicEstimatorGpsOrigin.cosLat0 * dLonRad;

  *posX = northMeters;
  *posY = -eastMeters;

  return isfinite(*posX) && isfinite(*posY);
}

static bool
kinematicEstimatorGpsToLocalAltitude(const gpsSolutionData_t *gpsSolution,
                                     float *posZ) {
  *posZ = kinematicEstimatorCentimetersToMeters((float)gpsSolution->llh.altCm) -
          kinematicEstimatorGpsOrigin.altMeters;

  return isfinite(*posZ);
}

static bool
kinematicEstimatorGpsToLocalVelocity(const gpsSolutionData_t *gpsSolution,
                                     float *velX, float *velY) {
  const float speedMetersPerSecond =
      kinematicEstimatorCentimetersToMeters((float)gpsSolution->groundSpeed);
  const float courseRad = DECIDEGREES_TO_RADIANS(gpsSolution->groundCourse);

  *velX = speedMetersPerSecond * cos_approx(-courseRad);
  *velY = speedMetersPerSecond * sin_approx(-courseRad);

  return isfinite(*velX) && isfinite(*velY);
}
#endif

static float kinematicEstimatorGetDistanceVarianceScale(int32_t distanceCm) {
  if (distanceCm > 0 && rangefinderMaxRangeCm > 0) {
    const float distanceNorm =
        constrainf(distanceCm / (float)rangefinderMaxRangeCm, 0.0f, 1.0f);

    return 1.0f + DISTANCE_VARIANCE_GAIN * distanceNorm * distanceNorm;
  }

  return 1.0f;
}

static float kinematicEstimatorGetRangefinderVarianceScale(
    const rangefinderMeasurement_t *rangefinderMeasurement) {
  float strengthScale = 1.0f;

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
  const float strengthNorm =
      constrainf(rangefinderMeasurement->distStrength / 255.0f, 0.0f, 1.0f);

  strengthScale = 1.0f + RANGEFINDER_STRENGTH_GAIN * (1.0f - strengthNorm);
#endif

  return constrainf(strengthScale * kinematicEstimatorGetDistanceVarianceScale(
                                        rangefinderMeasurement->rawAltitudeCm),
                    1.0f, RANGEFINDER_VARIANCE_SCALE_MAX);
}

static float kinematicEstimatorGetOpticalflowVarianceScale(
    const opticalflowMeasurement_t *opticalflowMeasurement,
    const rangefinderMeasurement_t *rangefinderMeasurement) {
  float qualityScale = 1.0f;

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
  const float qualityNorm =
      constrainf(opticalflowMeasurement->flowQuality / 255.0f, 0.0f, 1.0f);
  const float qualityError = 1.0f - qualityNorm;

  qualityScale = 1.0f + OPTICALFLOW_QUALITY_GAIN * qualityError * qualityError;
#else
  UNUSED(opticalflowMeasurement);
#endif

  return constrainf(qualityScale * kinematicEstimatorGetDistanceVarianceScale(
                                       rangefinderMeasurement->rawAltitudeCm),
                    1.0f, OPTICALFLOW_VARIANCE_SCALE_MAX);
}

void kinematicEstimatorInit(void) {
  kinematicFilterInit(&kinematicEstimator);
#ifdef USE_EKF_BARO
  kinematicEstimatorInvalidateBaro();
#endif
#ifdef USE_EKF_GPS
  kinematicEstimatorResetGpsOrigin();
#endif
}

void kinematicEstimatorReset(void) {
  kinematicFilterReset(&kinematicEstimator);
#ifdef USE_EKF_BARO
  kinematicEstimatorInvalidateBaro();
#endif
#ifdef USE_EKF_GPS
  kinematicEstimatorResetGpsOrigin();
#endif
}

void kinematicEstimatorResetState(const kinematicState_t *state) {
  if (!state) {
    kinematicEstimatorReset();
    return;
  }

  kinematicFilterResetState(&kinematicEstimator, state);

#ifdef USE_EKF_BARO
  kinematicEstimatorResetBaroHistory();
  kinematicEstimatorBaroState.needsBiasReset = false;
#endif
}

void kinematicEstimatorOnArm(void) {
  if (!kinematicEstimatorResetOnArmEnabled()) {
    return;
  }

#ifdef USE_EKF_GPS
  kinematicEstimatorResetGpsOrigin();
#endif

  kinematicEstimatorResetPositionVelocityState();

#ifdef USE_EKF_BARO
  kinematicEstimatorInvalidateBaro();
#endif
}

void kinematicEstimatorPredictFromImu(float accelBodyX, float accelBodyY,
                                      float accelBodyZ,
                                      const quaternion_t *attitudeQuat,
                                      float dt) {
  if (!attitudeQuat || dt <= 0.0f) {
    return;
  }

  kinematicFilterPredictInputs(
      &kinematicEstimator,
      kinematicEstimatorAccelToMetersPerSecondSquared(accelBodyX),
      kinematicEstimatorAccelToMetersPerSecondSquared(accelBodyY),
      kinematicEstimatorAccelToMetersPerSecondSquared(accelBodyZ),
      attitudeQuat->w, attitudeQuat->x, attitudeQuat->y, attitudeQuat->z, dt);
}

#ifdef USE_EKF_BARO
void kinematicEstimatorUpdateFromBaro(float baroAltitudeCm,
                                      timeUs_t currentTimeUs) {
  const float baroAltitudeMeters =
      kinematicEstimatorCentimetersToMeters(baroAltitudeCm);
  float varianceScale = 1.0f;

  if (!isfinite(baroAltitudeMeters)) {
    return;
  }

  if (kinematicEstimatorBaroState.needsBiasReset) {
    kinematicEstimator.state.baroBias =
        baroAltitudeMeters - kinematicEstimator.state.posZ;
    kinematicEstimatorBaroState.needsBiasReset = false;
  }

  if (kinematicEstimatorBaroState.hasHistory &&
      currentTimeUs > kinematicEstimatorBaroState.lastUpdateUs) {
    const float dt =
        (float)(currentTimeUs - kinematicEstimatorBaroState.lastUpdateUs) *
        1.0e-6f;

    if (dt > 0.0f) {
      const float baroRateMetersPerSecond =
          (baroAltitudeMeters -
           kinematicEstimatorBaroState.lastAltitudeMeters) /
          dt;
      const float roughnessMetersPerSecondSquared =
          fabsf(baroRateMetersPerSecond -
                kinematicEstimatorBaroState.lastRateMetersPerSecond) /
          dt;
      const float alpha =
          dt / (BARO_ROUGHNESS_LPF_TIME_CONSTANT_S + dt);

      kinematicEstimatorBaroState.roughnessMetersPerSecondSquared +=
          alpha * (roughnessMetersPerSecondSquared -
                   kinematicEstimatorBaroState.roughnessMetersPerSecondSquared);
      kinematicEstimatorBaroState.lastRateMetersPerSecond =
          baroRateMetersPerSecond;

      const float roughnessNorm =
          kinematicEstimatorBaroState.roughnessMetersPerSecondSquared /
          BARO_ROUGHNESS_REFERENCE_MPS2;

      varianceScale *=
          1.0f + BARO_ROUGHNESS_SCALE_GAIN * sq(roughnessNorm);
    }
  } else {
    kinematicEstimatorBaroState.lastRateMetersPerSecond =
        kinematicEstimator.state.velZ;
  }

  const float innovationMeters =
      baroAltitudeMeters -
      (kinematicEstimator.state.posZ + kinematicEstimator.state.baroBias);
  const float innovationNorm =
      innovationMeters / BARO_INNOVATION_SCALE_METERS;

  varianceScale *=
      1.0f + BARO_INNOVATION_SCALE_GAIN * sq(innovationNorm);
  varianceScale =
      constrainf(varianceScale, 1.0f, BARO_VARIANCE_SCALE_MAX);

  kinematicFilterUpdateBaroAltitude(&kinematicEstimator, baroAltitudeMeters,
                                    varianceScale);

  kinematicEstimatorBaroState.hasHistory = true;
  kinematicEstimatorBaroState.lastAltitudeMeters = baroAltitudeMeters;
  kinematicEstimatorBaroState.lastUpdateUs = currentTimeUs;
}
#endif

void kinematicEstimatorUpdateFromOpticalflow(
    const opticalflowMeasurement_t *opticalflowMeasurement,
    const rangefinderMeasurement_t *rangefinderMeasurement,
    const quaternion_t *attitudeQuat) {
  if (!opticalflowMeasurement || !rangefinderMeasurement || !attitudeQuat) {
    return;
  }

  if (!opticalflowMeasurement->isHealthy ||
      !rangefinderMeasurement->isHealthy ||
      rangefinderMeasurement->calculatedAltitudeCm < 0) {
    return;
  }

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
  if (opticalflowMeasurement->flowStatus == 0 ||
      rangefinderMeasurement->distStatus == 0) {
    return;
  }
#endif

  const float altitudeCm = (float)rangefinderMeasurement->calculatedAltitudeCm;
  const float opticalflowVarianceScale =
      kinematicEstimatorGetOpticalflowVarianceScale(opticalflowMeasurement,
                                                    rangefinderMeasurement);

  kinematicFilterUpdateFlowVelocity(
      &kinematicEstimator,
      kinematicEstimatorOpticalflowToMetersPerSecond(
          (float)opticalflowMeasurement->velX, altitudeCm),
      kinematicEstimatorOpticalflowToMetersPerSecond(
          (float)opticalflowMeasurement->velY, altitudeCm),
      attitudeQuat, opticalflowVarianceScale);
}

void kinematicEstimatorUpdateFromRangefinder(
    const rangefinderMeasurement_t *rangefinderMeasurement) {
  if (!rangefinderMeasurement || !rangefinderMeasurement->isHealthy ||
      rangefinderMeasurement->calculatedAltitudeCm < 0) {
    return;
  }

#ifdef USE_RANGEFINDER_OPTFLOW_MTF
  if (rangefinderMeasurement->distStatus == 0) {
    return;
  }
#endif

  const float rangefinderVarianceScale =
      kinematicEstimatorGetRangefinderVarianceScale(rangefinderMeasurement);

  kinematicFilterUpdatePositionZ(
      &kinematicEstimator,
      kinematicEstimatorCentimetersToMeters(
          (float)rangefinderMeasurement->calculatedAltitudeCm),
      rangefinderVarianceScale);
}

#ifdef USE_EKF_GPS
void kinematicEstimatorUpdateFromGps(const gpsSolutionData_t *gpsSolution) {
  float posX;
  float posY;
  float posZ;
  float velX;
  float velY;
  const float speedMetersPerSecond =
      gpsSolution ? kinematicEstimatorCentimetersToMeters(
                        (float)gpsSolution->groundSpeed)
                  : 0.0f;

  if (!gpsSolution || !sensors(SENSOR_GPS) || !STATE(GPS_FIX) ||
      !gpsIsHealthy()) {
    return;
  }

  if (gpsSolution->numSat < GPS_MIN_SAT_COUNT ||
      gpsSolution->navIntervalMs == 0) {
    return;
  }

  if (!kinematicEstimatorGpsOrigin.isInitialized) {
    kinematicEstimatorInitializeGpsOrigin(gpsSolution);
  }

  if (kinematicEstimatorGpsToLocalPosition(gpsSolution, &posX, &posY)) {
    kinematicFilterUpdateGpsPosition(
        &kinematicEstimator, posX, posY,
        kinematicEstimatorGetGpsPositionVarianceScale(gpsSolution));
  }

  if (kinematicEstimatorGpsToLocalAltitude(gpsSolution, &posZ)) {
    kinematicFilterUpdateGpsAltitude(
        &kinematicEstimator, posZ,
        kinematicEstimatorGetGpsAltitudeVarianceScale(gpsSolution));
  }

  if (speedMetersPerSecond < GPS_COURSE_MIN_SPEED_MPS) {
    return;
  }

  if (kinematicEstimatorGpsToLocalVelocity(gpsSolution, &velX, &velY)) {
    kinematicFilterUpdateGpsVelocity(
        &kinematicEstimator, velX, velY,
        kinematicEstimatorGetGpsVelocityVarianceScale(gpsSolution));
  }
}
#endif

const kinematicFilter_t *kinematicEstimatorGetFilter(void) {
  return &kinematicEstimator;
}

const kinematicState_t *kinematicEstimatorGetState(void) {
  return &kinematicEstimator.state;
}
