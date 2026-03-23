#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ekf/kinematic.h"

#define KINEMATIC_STATE_COVARIANCE_DIM (KINEMATIC_ERROR_DIM * KINEMATIC_ERROR_DIM)
#define KINEMATIC_OBS_COVARIANCE_DIM_2 (KINEMATIC_OBS_DIM_2 * KINEMATIC_OBS_DIM_2)
#define KINEMATIC_OBS_COVARIANCE_DIM_3 (KINEMATIC_OBS_DIM_3 * KINEMATIC_OBS_DIM_3)
#define KINEMATIC_OBS_COVARIANCE_DIM_4 (KINEMATIC_OBS_DIM_4 * KINEMATIC_OBS_DIM_4)

typedef enum {
    KINEMATIC_STATE_POS_X = 0,
    KINEMATIC_STATE_POS_Y,
    KINEMATIC_STATE_POS_Z,
    KINEMATIC_STATE_VEL_X,
    KINEMATIC_STATE_VEL_Y,
    KINEMATIC_STATE_VEL_Z,
    KINEMATIC_STATE_BARO_BIAS,
    KINEMATIC_STATE_ACCEL_BIAS_X,
    KINEMATIC_STATE_ACCEL_BIAS_Y,
    KINEMATIC_STATE_ACCEL_BIAS_Z,
} kinematicStateIndex_e;

typedef union {
    float raw[KINEMATIC_STATE_DIM];
    struct {
        float posX;
        float posY;
        float posZ;
        float velX;
        float velY;
        float velZ;
        float baroBias;
        float accelBiasX;
        float accelBiasY;
        float accelBiasZ;
    };
} kinematicState_t;

typedef union {
    float raw[KINEMATIC_CONTROL_DIM];
    struct {
        float accelBodyX;
        float accelBodyY;
        float accelBodyZ;
        float attitudeQuatW;
        float attitudeQuatX;
        float attitudeQuatY;
        float attitudeQuatZ;
    };
} kinematicControl_t;

typedef union {
    float raw[KINEMATIC_OBS_DIM_2];
    struct {
        float posZ;
    };
} kinematicObs2_t;

typedef union {
    float raw[KINEMATIC_OBS_DIM_3];
    struct {
        float baroAltitude;
    };
} kinematicObs3_t;

typedef union {
    float raw[KINEMATIC_OBS_DIM_4];
    struct {
        float velX;
        float velY;
    };
} kinematicObs4_t;

typedef union {
    float raw[KINEMATIC_EXTRA_DIM_4];
    struct {
        float w;
        float x;
        float y;
        float z;
    };
} kinematicQuaternion_t;

typedef struct kinematicFilter_s {
    kinematicState_t state;
    float P[KINEMATIC_STATE_COVARIANCE_DIM];
    float Q[KINEMATIC_STATE_COVARIANCE_DIM];
    float R2[KINEMATIC_OBS_COVARIANCE_DIM_2];
    float R3[KINEMATIC_OBS_COVARIANCE_DIM_3];
    float R4[KINEMATIC_OBS_COVARIANCE_DIM_4];
} kinematicFilter_t;

void kinematicFilterInit(kinematicFilter_t *filter);
void kinematicFilterReset(kinematicFilter_t *filter);
void kinematicFilterResetState(kinematicFilter_t *filter, const kinematicState_t *state);

void kinematicFilterSetStateCovarianceDiagonal(kinematicFilter_t *filter, float variance);
void kinematicFilterSetProcessNoiseDiagonal(kinematicFilter_t *filter, float variance);
void kinematicFilterSetPositionZVariance(kinematicFilter_t *filter, float variance);
void kinematicFilterSetBaroAltitudeVariance(kinematicFilter_t *filter, float variance);
void kinematicFilterSetFlowVelocityVariances(kinematicFilter_t *filter, float velXVariance, float velYVariance);

void kinematicFilterPredictRaw(kinematicFilter_t *filter, const float control[KINEMATIC_CONTROL_DIM], float dt);
void kinematicFilterPredict(kinematicFilter_t *filter, const kinematicControl_t *control, float dt);
void kinematicFilterPredictInputs(kinematicFilter_t *filter,
    float accelBodyX, float accelBodyY, float accelBodyZ,
    float attitudeQuatW, float attitudeQuatX, float attitudeQuatY, float attitudeQuatZ,
    float dt);

void kinematicFilterUpdatePositionZRaw(kinematicFilter_t *filter, const float measurement[KINEMATIC_OBS_DIM_2]);
void kinematicFilterUpdateBaroAltitudeRaw(kinematicFilter_t *filter, const float measurement[KINEMATIC_OBS_DIM_3]);
void kinematicFilterUpdateFlowVelocityRaw(kinematicFilter_t *filter, const float measurement[KINEMATIC_OBS_DIM_4], const float flowQuaternion[KINEMATIC_EXTRA_DIM_4]);

void kinematicFilterUpdatePositionZ(kinematicFilter_t *filter, float posZ, float varianceScale);
static inline void kinematicFilterUpdatePositionZNominal(kinematicFilter_t *filter, float posZ)
{
    kinematicFilterUpdatePositionZ(filter, posZ, 1.0f);
}
void kinematicFilterUpdateBaroAltitude(kinematicFilter_t *filter, float baroAltitude);
void kinematicFilterUpdateFlowVelocity(kinematicFilter_t *filter, float velX, float velY, const kinematicQuaternion_t *flowQuaternion);

#ifdef __cplusplus
}
#endif
