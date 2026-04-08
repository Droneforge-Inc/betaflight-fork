#include "platform.h"

#include <math.h>
#include <string.h>

#include "common/maths.h"

#include "config/feature.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/droneforge_policy.h"
#include "flight/droneforge_policy_runtime.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#if defined(USE_EKF) && defined(USE_OPTICALFLOW) && defined(USE_RANGEFINDER)

#include "flight/kinematic_estimator.h"

#include "sensors/gyro.h"
#include "sensors/opticalflow.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

#define DRONEFORGE_POLICY_INTERVAL_US 10000
#define DRONEFORGE_POLICY_RESET_ACTION 0.0f

typedef struct {
    bool runtimeReady;
    bool active;
    timeUs_t lastStepUs;
    float frameCos;
    float frameSin;
    float previousAction[DRONEFORGE_POLICY_ACTION_DIM];
    kinematicState_t origin;
} droneforgePolicyState_t;

static droneforgePolicyState_t policyState;

static bool droneforgePolicyRequested(void)
{
    return policyState.runtimeReady
        && ARMING_FLAG(ARMED)
        && IS_RC_MODE_ACTIVE(BOXUSER1)
        && !failsafeIsActive()
        && featureIsEnabled(FEATURE_RANGEFINDER)
        && featureIsEnabled(FEATURE_OPTICALFLOW)
        && sensors(SENSOR_RANGEFINDER)
        && sensors(SENSOR_OPTICALFLOW)
        && rangefinderIsHealthy()
        && opticalflowIsHealthy()
        && getMixerMode() == MIXER_QUADX
        && getMotorCount() == 4;
}

static void droneforgePolicyRotateWorldToLocal(float x, float y, float *outX, float *outY)
{
    *outX = policyState.frameCos * x + policyState.frameSin * y;
    *outY = -policyState.frameSin * x + policyState.frameCos * y;
}

static void droneforgePolicyClearAction(float action[DRONEFORGE_POLICY_ACTION_DIM])
{
    for (int i = 0; i < DRONEFORGE_POLICY_ACTION_DIM; i++) {
        action[i] = DRONEFORGE_POLICY_RESET_ACTION;
    }
}

static void droneforgePolicyEnter(timeUs_t currentTimeUs)
{
    const kinematicState_t *state = kinematicEstimatorGetState();
    const float yaw = atan2f(rMat[1][0], rMat[0][0]);

    policyState.origin = *state;
    policyState.frameCos = cosf(yaw);
    policyState.frameSin = sinf(yaw);
    policyState.lastStepUs = currentTimeUs - DRONEFORGE_POLICY_INTERVAL_US;
    policyState.active = true;

    droneforgePolicyClearAction(policyState.previousAction);
    droneforgePolicyRuntimeReset();
    pidResetIterm();
}

static void droneforgePolicyBuildObservation(droneforgePolicyObservation_t *observation)
{
    const kinematicState_t *state = kinematicEstimatorGetState();
    float *obs = observation->values;
    float dx = state->posX - policyState.origin.posX;
    float dy = state->posY - policyState.origin.posY;
    float vx = state->velX;
    float vy = state->velY;
    float localRot[3][3];

    droneforgePolicyRotateWorldToLocal(dx, dy, &obs[0], &obs[1]);
    obs[2] = state->posZ - policyState.origin.posZ;

    for (int column = 0; column < 3; column++) {
        const float worldX = rMat[0][column];
        const float worldY = rMat[1][column];

        localRot[0][column] = policyState.frameCos * worldX + policyState.frameSin * worldY;
        localRot[1][column] = -policyState.frameSin * worldX + policyState.frameCos * worldY;
        localRot[2][column] = rMat[2][column];
    }

    obs[3] = localRot[0][0];
    obs[4] = localRot[0][1];
    obs[5] = localRot[0][2];
    obs[6] = localRot[1][0];
    obs[7] = localRot[1][1];
    obs[8] = localRot[1][2];
    obs[9] = localRot[2][0];
    obs[10] = localRot[2][1];
    obs[11] = localRot[2][2];

    droneforgePolicyRotateWorldToLocal(vx, vy, &obs[12], &obs[13]);
    obs[14] = state->velZ;

    obs[15] = DEGREES_TO_RADIANS(gyro.gyroADCf[X]);
    obs[16] = DEGREES_TO_RADIANS(gyro.gyroADCf[Y]);
    obs[17] = DEGREES_TO_RADIANS(gyro.gyroADCf[Z]);

    memcpy(&obs[18], policyState.previousAction, sizeof(policyState.previousAction));
}

static float droneforgePolicyMapMotor(float value)
{
    const float outputLow = getMotorOutputLow();
    const float outputHigh = getMotorOutputHigh();
    const float normalized = constrainf(0.5f * (value + 1.0f), 0.0f, 1.0f);

    return outputLow + normalized * (outputHigh - outputLow);
}

static void droneforgePolicyApplyAction(const float action[DRONEFORGE_POLICY_ACTION_DIM])
{
    motor[0] = droneforgePolicyMapMotor(action[1]);
    motor[1] = droneforgePolicyMapMotor(action[0]);
    motor[2] = droneforgePolicyMapMotor(action[2]);
    motor[3] = droneforgePolicyMapMotor(action[3]);
}

void droneforgePolicyInit(void)
{
    memset(&policyState, 0, sizeof(policyState));
    policyState.runtimeReady = droneforgePolicyRuntimeInit();
}

void droneforgePolicyReset(void)
{
    policyState.active = false;
    policyState.lastStepUs = 0;
    memset(policyState.previousAction, 0, sizeof(policyState.previousAction));
    pidResetIterm();
}

bool droneforgePolicyRun(timeUs_t currentTimeUs)
{
    droneforgePolicyObservation_t observation;
    float action[DRONEFORGE_POLICY_ACTION_DIM];

    if (!droneforgePolicyRequested()) {
        if (policyState.active) {
            droneforgePolicyReset();
        }
        return false;
    }

    if (!policyState.active) {
        droneforgePolicyEnter(currentTimeUs);
    }

    if ((currentTimeUs - policyState.lastStepUs) >= DRONEFORGE_POLICY_INTERVAL_US) {
        droneforgePolicyBuildObservation(&observation);
        if (!droneforgePolicyRuntimeStep(currentTimeUs, &observation, action)) {
            droneforgePolicyReset();
            return false;
        }
        memcpy(policyState.previousAction, action, sizeof(policyState.previousAction));
        policyState.lastStepUs = currentTimeUs;
    }

    droneforgePolicyApplyAction(policyState.previousAction);
    return true;
}

#else

void droneforgePolicyInit(void)
{
}

void droneforgePolicyReset(void)
{
}

bool droneforgePolicyRun(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    return false;
}

#endif
