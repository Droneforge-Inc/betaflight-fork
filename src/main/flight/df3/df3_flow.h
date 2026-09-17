/* Timestamped angular history for optical-flow interval compensation. */
#pragma once
#include "df3_core.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { DF3_GYRO_HISTORY_CAPACITY = 128 };
typedef struct {
    uint64_t us;
    float gyro[3];
} df3GyroSample_t;
typedef struct {
    df3GyroSample_t samples[DF3_GYRO_HISTORY_CAPACITY];
    uint16_t first, count;
} df3GyroHistory_t;
typedef struct {
    uint64_t startUs, endUs, midpointUs;
    float averageGyro[3];
    float rawBodyVelocity[2], rotationCorrection[2], leverCorrection[2], correctedBodyVelocity[2];
} df3FlowResult_t;

void df3GyroHistoryReset(df3GyroHistory_t *history);
bool df3GyroHistoryPush(df3GyroHistory_t *history, uint64_t us, const float gyro[3]);
/* One left-held interval. Copies its rate so callers never retain ring pointers.
 * tail=true permits at most 10 ms of current-time projection beyond the last
 * sample; covariance/window fusion always requires measured end coverage. */
bool df3GyroHistoryInterval(const df3GyroHistory_t *history, uint64_t startUs, uint64_t endUs, bool tail, float gyro[3],
                            uint64_t *nextUs, uint64_t *sampleUs);
/* Optional caller-owned physical-index hint for sequential replay. Every use
 * revalidates ring membership and the timestamp bracket, including after
 * overwrite/reset. No sample pointer survives the call. Initialize to
 * UINT16_MAX (any other value is also safe); only success updates the hint.
 * The next-call guess changes lookup cost, never accepted intervals or math. */
bool df3GyroHistoryIntervalHint(const df3GyroHistory_t *history, uint64_t startUs, uint64_t endUs, bool tail,
                                float gyro[3], uint64_t *nextUs, uint64_t *sampleUs, uint16_t *physicalHint);
bool df3GyroHistoryAverage(const df3GyroHistory_t *history, uint64_t startUs, uint64_t endUs, float average[3]);
/* gyro FRD radians/s; normalized flow in the existing SDK forward/right
 * convention at 1 m, clearance in m. rotationScale is a sensor calibration
 * (legacy SDK 0.758; rendered pinhole nominally 1). sensorOffset is body FRD
 * from COM; use zero for DF2 parity.
 * Exact compensation requires source acquisition intervals mapped to the FC
 * clock. Receipt time must not be passed as if it were acquisition time. */
bool df3FlowCompensate(const df3GyroHistory_t *history, uint64_t startUs, uint64_t endUs, const float normalizedFlow[2],
                       float clearance, float rotationScale, const float sensorOffset[3], df3FlowResult_t *result);

/* For zero sensor lever arm: z = height*flow - C*measuredGyro,
 * h(x) = (R_LB^T v_L).xy - C*b_g, C = [[0,k,0],[-k,0,0]],
 * k = height*rotationScale. The third row is deliberately unobserved.
 * Keeping bias and attitude in H avoids pretending corrected velocity is an
 * independent world-frame measurement. No horizontal position is observed. */
bool df3FlowObserve(const float x[DF3_NX], float rotationGain, float h[3], float H[3 * DF3_NE]);

/* Same measurement with the lens offset from COM in body FRD metres.
 * rotationGain remains height*rotationScale (excludes the lever arm).
 * The gyro-bias residual includes -bias x offset, including yaw coupling. */
bool df3FlowObserveOffset(const float x[DF3_NX], float rotationGain, const float sensorOffset[3], float h[3],
                          float H[3 * DF3_NE]);

/* Rounded flow describes [z-step/2,z+step/2], in corrected body m/s.
 * Approximate the two scalar interval posteriors by equivalent Gaussian
 * observations. R retains the caller's noise/robustness policy; this never
 * increases its information. step=0 preserves the point-observation path.
 * Gyro compensation shifts the bin center, not its width. No dequantized
 * ground-truth velocity is inferred. R must be diagonal and positive. */
bool df3FlowQuantizedObservation(float step, const float prior[2], float z[3], const float h[3], float H[3 * DF3_NE],
                                 float R[9]);

#ifdef __cplusplus
}
#endif
