/* Allocation-free local-NED outer loop. Betaflight retains angle/rate control. */
#pragma once
#include "df3_estimator.h"
#include "df3_reference.h"

// Accumulated position error, m*s. Acceleration/throttle limits and
// conditional antiwindup still apply.
#define DF3_CONTROL_XY_INTEGRAL_LIMIT_M_S 5.0f
#define DF3_CONTROL_Z_INTEGRAL_LIMIT_M_S 3.0f

// Controller cadence and fallback policy; timestamps are microseconds.
enum { DF3_CONTROL_PERIOD_US = 4000, DF3_CONTROL_MAX_GAP_US = 20000, DF3_CONTROL_REFERENCE_HOLD_US = 500000 };
#define DF3_CONTROL_INITIAL_DT_S .004f
#define DF3_CONTROL_LAND_SPEED_M_S .25f
#define DF3_CONTROL_VERTICAL_ACCEL_LIMIT_M_S2 4
#define DF3_CONTROL_MIN_THROTTLE .05f
#define DF3_CONTROL_MAX_THROTTLE .85f

typedef struct {
    // Fixed feedback gains from the aircraft configuration, in SI units.
    float kp[3], kv[3], ki[3], integralLimit[3];
    float hoverThrottle, accelToThrottle, maxTiltRad;
} df3ControlConfig_t;
typedef enum {
    DF3_CONTROL_OFF,
    DF3_CONTROL_WAIT,
    DF3_CONTROL_TRACK,
    DF3_CONTROL_HOLD,
    DF3_CONTROL_LAND,
    DF3_CONTROL_FAULT
} df3ControlMode_e;
typedef struct {
    float angleDeg[2], yawRateDeg, throttle, acceleration[3];
    df3ControlMode_e mode;
    bool authority;
} df3ControlOutput_t;
typedef struct {
    float integral[3];
    df3Reference_t fallback;
    uint64_t lastUs, lostReferenceUs;
    bool engaged, fault;
} df3Control_t;

void df3ControlReset(df3Control_t *controller);
bool df3ControlConfigValid(const df3ControlConfig_t *config);
void df3ControlStep(df3Control_t *controller, const df3ControlConfig_t *config, uint64_t nowUs, bool selected,
                    bool armed, bool permit, bool nativeOverride, bool imuFresh, bool rangeFresh, bool flowFresh,
                    const df3Estimate_t *estimate, const df3Reference_t *reference, df3ControlOutput_t *out);
