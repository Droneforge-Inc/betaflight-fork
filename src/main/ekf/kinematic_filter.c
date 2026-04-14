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

#include <string.h>

#include "ekf/kinematic_filter.h"

static const float kinematicGpsAltitudeVarianceDefault = 36.0f;
static const float kinematicGpsPositionVarianceDefault = 9.0f;
static const float kinematicGpsVelocityVarianceDefault = 1.0f;
static const float kinematicBaroAltitudeVarianceDefault = 0.25f;

static void kinematicFilterSetCovarianceDiagonal(float *matrix, int dimension,
                                                 float variance) {
  memset(matrix, 0, (size_t)(dimension * dimension) * sizeof(float));

  for (int i = 0; i < dimension; i++) {
    matrix[(i * dimension) + i] = variance;
  }
}

static void kinematicFilterSetCovarianceDiagonal2(float *matrix,
                                                  float variance0,
                                                  float variance1) {
  memset(matrix, 0, 4 * sizeof(float));
  matrix[0] = variance0;
  matrix[3] = variance1;
}

static float kinematicFilterSanitizeVarianceScale(float varianceScale) {
  return (varianceScale > 0.0f) ? varianceScale : 1.0f;
}

void kinematicFilterInit(kinematicFilter_t *filter) {
  memset(filter, 0, sizeof(*filter));

  kinematicFilterSetStateCovarianceDiagonal(filter, 1.0f);
  kinematicFilterSetProcessNoiseDiagonal(filter, 0.0f);
  kinematicFilterSetPositionZVariance(filter, 1e-3f);
  kinematicFilterSetBaroAltitudeVariance(filter,
                                         kinematicBaroAltitudeVarianceDefault);
  kinematicFilterSetFlowVelocityVariances(filter, 1e-2f, 1e-2f);
  kinematicFilterSetGpsAltitudeVariance(filter,
                                        kinematicGpsAltitudeVarianceDefault);
  kinematicFilterSetGpsPositionVariances(filter,
                                         kinematicGpsPositionVarianceDefault,
                                         kinematicGpsPositionVarianceDefault);
  kinematicFilterSetGpsVelocityVariances(filter,
                                         kinematicGpsVelocityVarianceDefault,
                                         kinematicGpsVelocityVarianceDefault);
}

void kinematicFilterReset(kinematicFilter_t *filter) {
  kinematicFilterInit(filter);
}

void kinematicFilterResetState(kinematicFilter_t *filter,
                               const kinematicState_t *state) {
  memcpy(filter->state.raw, state->raw, sizeof(filter->state.raw));
  kinematic_normalize_state(filter->state.raw);
}

void kinematicFilterSetStateCovarianceDiagonal(kinematicFilter_t *filter,
                                               float variance) {
  kinematicFilterSetCovarianceDiagonal(filter->P, KINEMATIC_ERROR_DIM,
                                       variance);
}

void kinematicFilterSetProcessNoiseDiagonal(kinematicFilter_t *filter,
                                            float variance) {
  kinematicFilterSetCovarianceDiagonal(filter->Q, KINEMATIC_ERROR_DIM,
                                       variance);
}

void kinematicFilterSetPositionZVariance(kinematicFilter_t *filter,
                                         float variance) {
  kinematicFilterSetCovarianceDiagonal(filter->R2, KINEMATIC_OBS_DIM_2,
                                       variance);
}

void kinematicFilterSetBaroAltitudeVariance(kinematicFilter_t *filter,
                                            float variance) {
  kinematicFilterSetCovarianceDiagonal(filter->R3, KINEMATIC_OBS_DIM_3,
                                       variance);
}

void kinematicFilterSetFlowVelocityVariances(kinematicFilter_t *filter,
                                             float velXVariance,
                                             float velYVariance) {
  memset(filter->R4, 0, sizeof(filter->R4));
  filter->R4[0] = velXVariance;
  filter->R4[3] = velYVariance;
}

void kinematicFilterSetGpsAltitudeVariance(kinematicFilter_t *filter,
                                           float variance) {
  kinematicFilterSetCovarianceDiagonal(filter->R5, KINEMATIC_OBS_DIM_5,
                                       variance);
}

void kinematicFilterSetGpsPositionVariances(kinematicFilter_t *filter,
                                            float posXVariance,
                                            float posYVariance) {
  kinematicFilterSetCovarianceDiagonal2(filter->R6, posXVariance, posYVariance);
}

void kinematicFilterSetGpsVelocityVariances(kinematicFilter_t *filter,
                                            float velXVariance,
                                            float velYVariance) {
  kinematicFilterSetCovarianceDiagonal2(filter->R7, velXVariance, velYVariance);
}

void kinematicFilterPredictRaw(kinematicFilter_t *filter,
                               const float control[KINEMATIC_CONTROL_DIM],
                               float dt) {
  float controlCopy[KINEMATIC_CONTROL_DIM];

  memcpy(controlCopy, control, sizeof(controlCopy));
#if KINEMATIC_HAS_SYMBOLIC_Q
  kinematic_predict(filter->state.raw, filter->P, controlCopy, dt);
#else
  kinematic_predict(filter->state.raw, filter->P, filter->Q, controlCopy, dt);
#endif
}

void kinematicFilterPredict(kinematicFilter_t *filter,
                            const kinematicControl_t *control, float dt) {
  kinematicFilterPredictRaw(filter, control->raw, dt);
}

void kinematicFilterPredictInputs(kinematicFilter_t *filter, float accelBodyX,
                                  float accelBodyY, float accelBodyZ,
                                  float attitudeQuatW, float attitudeQuatX,
                                  float attitudeQuatY, float attitudeQuatZ,
                                  float dt) {
  kinematicControl_t control = {
      .accelBodyX = accelBodyX,
      .accelBodyY = accelBodyY,
      .accelBodyZ = accelBodyZ,
      .attitudeQuatW = attitudeQuatW,
      .attitudeQuatX = attitudeQuatX,
      .attitudeQuatY = attitudeQuatY,
      .attitudeQuatZ = attitudeQuatZ,
  };

  kinematicFilterPredict(filter, &control, dt);
}

void kinematicFilterUpdatePositionZRaw(
    kinematicFilter_t *filter, const float measurement[KINEMATIC_OBS_DIM_2]) {
  float measurementCopy[KINEMATIC_OBS_DIM_2];

  memcpy(measurementCopy, measurement, sizeof(measurementCopy));
  kinematic_update_2(filter->state.raw, filter->P, measurementCopy, filter->R2,
                     NULL);
}

void kinematicFilterUpdateBaroAltitudeRaw(
    kinematicFilter_t *filter, const float measurement[KINEMATIC_OBS_DIM_3]) {
  float measurementCopy[KINEMATIC_OBS_DIM_3];

  memcpy(measurementCopy, measurement, sizeof(measurementCopy));
  kinematic_update_3(filter->state.raw, filter->P, measurementCopy, filter->R3,
                     NULL);
}

void kinematicFilterUpdateFlowVelocityRaw(
    kinematicFilter_t *filter, const float measurement[KINEMATIC_OBS_DIM_4],
    const float flowQuaternion[KINEMATIC_EXTRA_DIM_4]) {
  float measurementCopy[KINEMATIC_OBS_DIM_4];
  float extraArgs[KINEMATIC_EXTRA_DIM_4];

  memcpy(measurementCopy, measurement, sizeof(measurementCopy));
  memcpy(extraArgs, flowQuaternion, sizeof(extraArgs));
  kinematic_update_4(filter->state.raw, filter->P, measurementCopy, filter->R4,
                     extraArgs);
}

void kinematicFilterUpdateGpsAltitudeRaw(
    kinematicFilter_t *filter, const float measurement[KINEMATIC_OBS_DIM_5]) {
  float measurementCopy[KINEMATIC_OBS_DIM_5];

  memcpy(measurementCopy, measurement, sizeof(measurementCopy));
  kinematic_update_5(filter->state.raw, filter->P, measurementCopy, filter->R5,
                     NULL);
}

void kinematicFilterUpdateGpsPositionRaw(
    kinematicFilter_t *filter, const float measurement[KINEMATIC_OBS_DIM_6]) {
  float measurementCopy[KINEMATIC_OBS_DIM_6];

  memcpy(measurementCopy, measurement, sizeof(measurementCopy));
  kinematic_update_6(filter->state.raw, filter->P, measurementCopy, filter->R6,
                     NULL);
}

void kinematicFilterUpdateGpsVelocityRaw(
    kinematicFilter_t *filter, const float measurement[KINEMATIC_OBS_DIM_7]) {
  float measurementCopy[KINEMATIC_OBS_DIM_7];

  memcpy(measurementCopy, measurement, sizeof(measurementCopy));
  kinematic_update_7(filter->state.raw, filter->P, measurementCopy, filter->R7,
                     NULL);
}

void kinematicFilterUpdatePositionZ(kinematicFilter_t *filter, float posZ,
                                    float varianceScale) {
  kinematicObs2_t measurement = {.posZ = posZ};
  float scaledR[KINEMATIC_OBS_COVARIANCE_DIM_2];

  scaledR[0] =
      filter->R2[0] * kinematicFilterSanitizeVarianceScale(varianceScale);

  kinematic_update_2(filter->state.raw, filter->P, measurement.raw, scaledR,
                     NULL);
}

void kinematicFilterUpdateBaroAltitude(kinematicFilter_t *filter,
                                       float baroAltitude,
                                       float varianceScale) {
  kinematicObs3_t measurement = {
      .baroAltitude = baroAltitude,
  };
  float scaledR[KINEMATIC_OBS_COVARIANCE_DIM_3];

  scaledR[0] =
      filter->R3[0] * kinematicFilterSanitizeVarianceScale(varianceScale);

  kinematic_update_3(filter->state.raw, filter->P, measurement.raw, scaledR,
                     NULL);
}

void kinematicFilterUpdateFlowVelocity(kinematicFilter_t *filter, float velX,
                                       float velY,
                                       const quaternion_t *flowQuaternion,
                                       float varianceScale) {
  kinematicObs4_t measurement = {
      .velX = velX,
      .velY = velY,
  };
  float scaledR[KINEMATIC_OBS_COVARIANCE_DIM_4] = {0};
  float extraArgs[KINEMATIC_EXTRA_DIM_4];
  const float sanitizedVarianceScale =
      kinematicFilterSanitizeVarianceScale(varianceScale);

  scaledR[0] = filter->R4[0] * sanitizedVarianceScale;
  scaledR[3] = filter->R4[3] * sanitizedVarianceScale;
  memcpy(extraArgs, flowQuaternion->raw, sizeof(extraArgs));

  kinematic_update_4(filter->state.raw, filter->P, measurement.raw, scaledR,
                     extraArgs);
}

void kinematicFilterUpdateGpsAltitude(kinematicFilter_t *filter,
                                      float gpsAltitude,
                                      float varianceScale) {
  kinematicObs5_t measurement = {
      .gpsAltitude = gpsAltitude,
  };
  float scaledR[KINEMATIC_OBS_COVARIANCE_DIM_5];

  scaledR[0] =
      filter->R5[0] * kinematicFilterSanitizeVarianceScale(varianceScale);

  kinematic_update_5(filter->state.raw, filter->P, measurement.raw, scaledR,
                     NULL);
}

void kinematicFilterUpdateGpsPosition(kinematicFilter_t *filter, float posX,
                                      float posY, float varianceScale) {
  kinematicObs6_t measurement = {
      .posX = posX,
      .posY = posY,
  };
  float scaledR[KINEMATIC_OBS_COVARIANCE_DIM_6] = {0};
  const float sanitizedVarianceScale =
      kinematicFilterSanitizeVarianceScale(varianceScale);

  scaledR[0] = filter->R6[0] * sanitizedVarianceScale;
  scaledR[3] = filter->R6[3] * sanitizedVarianceScale;

  kinematic_update_6(filter->state.raw, filter->P, measurement.raw, scaledR,
                     NULL);
}

void kinematicFilterUpdateGpsVelocity(kinematicFilter_t *filter, float velX,
                                      float velY, float varianceScale) {
  kinematicObs7_t measurement = {
      .velX = velX,
      .velY = velY,
  };
  float scaledR[KINEMATIC_OBS_COVARIANCE_DIM_7] = {0};
  const float sanitizedVarianceScale =
      kinematicFilterSanitizeVarianceScale(varianceScale);

  scaledR[0] = filter->R7[0] * sanitizedVarianceScale;
  scaledR[3] = filter->R7[3] * sanitizedVarianceScale;

  kinematic_update_7(filter->state.raw, filter->P, measurement.raw, scaledR,
                     NULL);
}
