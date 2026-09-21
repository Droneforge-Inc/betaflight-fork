/* Production: windowed observation/fusion contract.
 * df3_runtime.h selects 30 Hz; 100 Hz is retained for research comparisons. */
#pragma once
#include "df3_flow.h"
#include "df3_estimator.h"
#if defined(USE_DF3_MULTIRATE) && !defined(USE_DF3_RESUMABLE)
#error "Multirate DF3 requires the resumable transaction owner"
#endif

#ifndef DF3_FUSION_HZ
/* Historical default for standalone builds that omit df3_runtime.h. */
#define DF3_FUSION_HZ 100
#endif
#if DF3_FUSION_HZ != 100 && DF3_FUSION_HZ != 30
#error "DF3_FUSION_HZ must be 100 or 30"
#endif
enum {
    /* An even duration preserves the exact acquisition-window midpoint.
     * 33334 us is nominally 30 Hz (29.9994 Hz), without a fractional clock. */
    DF3_EPOCH_US = DF3_FUSION_HZ == 30 ? 33334 : 10000,
    DF3_RAW_IMU_MAX_GAP_US = 10000,
    /* Native range/flow arrivals remain independent of the fusion cadence.
     * Include one boundary/jitter allowance beyond ceil(epoch / 10 ms). */
    DF3_EPOCH_MTF_MAX = (DF3_EPOCH_US + 9999) / 10000 + 1,
    DF3_EPOCH_CAPACITY = 2 * DF3_EPOCH_MTF_MAX + 4
};
#ifndef DF3_STATIONARY_PERIOD_US
#define DF3_STATIONARY_PERIOD_US 50000
#endif
_Static_assert(DF3_STATIONARY_PERIOD_US >= DF3_EPOCH_US, "Stationary period must cover at least one epoch");
#if DF3_FUSION_HZ == 100
_Static_assert(DF3_STATIONARY_PERIOD_US % DF3_EPOCH_US == 0, "100 Hz stationary period must be whole epochs");
#endif

/* Small-block composition: rotation, bias sensitivity, and angular process Q.
 * No dense covariance is applied at raw sensor rate. */
typedef struct {
    float q[4], A[9], B[9], T[9], C[9], bb;
} df3Rotation_t;
void df3RotationReset(df3Rotation_t *rotation);
void df3RotationStep(df3Rotation_t *rotation, const float gyro[3], const float bias[3], float dt, bool noise);

/* Left-held filtered accelerometer integrated by duration over fixed bins.
 * The event center is exact; data[3..5] is the window mean, data[0] records
 * stationary eligibility. data[1..2] carry filtered force roughness (XY/Z),
 * in squared m/s^2 normalized to a 1 ms second difference; never gyro. */
typedef struct {
    uint64_t lastUs, startUs, attitudeBin, stationarySinceUs;
    float held[3], sum[3], rangeAnchor;
    float forceDerivative[3], forceRoughness[3], derivativeDt;
    uint32_t windows, gaps, invalid, partial;
    bool haveSample, stationary;
} df3ImuReducer_t;
void df3ImuReducerReset(df3ImuReducer_t *reducer);
/* Returns -1 for invalid/gapped input, 0 for no complete window, 1 for a
 * sealed event. Startup discards only the explicitly incomplete first bin. */
int df3ImuReducerPush(df3ImuReducer_t *reducer, uint64_t us, const float gyro[3], const float force[3], bool inactive,
                      bool externalStill, float range, df3Event_t *event);
bool df3AttitudeAdmit(df3ImuReducer_t *reducer, uint64_t us);

/* Core/window model helpers are portable, independently tested against dense
 * covariance composition and finite differences of the observation. */
void df3WindowObservation(const float x[DF3_NX], const float M[9], const float biasH[9], float h[3],
                          float H[3 * DF3_NE]);

/* Bounded continuation of the existing quantized-flow likelihood. Each slow
 * step integrates at most one five-point segment, never all 80 exponentials. */
typedef struct {
    unsigned axis, phase, segment, segments;
    float step, center, sd, lo, half, peak, mass, first, second, p, r, s;
    float observation[2], variance[2];
    bool unobserved[2];
} df3FlowQuantizedJob_t;
void df3FlowQuantizedStart(df3FlowQuantizedJob_t *job, float step);
/* -1 invalid, 0 yielded, 1 complete; outputs mutate only on completion. */
int df3FlowQuantizedResume(df3FlowQuantizedJob_t *job, const float prior[2], float z[3], const float h[3],
                           float H[3 * DF3_NE], float R[9]);
