/* Production: bounded estimator jobs and fusion transaction ownership.
 * df3_runtime.h enables resumable and multirate work together; non-multirate
 * branches remain available to standalone research comparisons.
 * An operation owns its core/workspace until completion or explicit reset.
 * Public estimates are published only after a complete fusion transaction. */
#pragma once
#include "df3_estimator.h"
#ifdef USE_DF3_BLACKBOX
#include "df3_blackbox.h"
#endif
#ifdef USE_DF3_MULTIRATE
#include "df3_multirate.h"
#endif

#ifdef USE_DF3_RESUMABLE
enum { DF3_SLICE_ROWS = 6 };
#ifdef USE_DF3_MULTIRATE
/* Three resumptions per pass, with boundaries matched to arithmetic cost.
 * Shared with the scheduler key mapping so each slice learns its own budget. */
enum {
    DF3_PREDICT_LEFT_FIRST_END = 5,
    DF3_PREDICT_LEFT_SECOND_END = 11,
    DF3_PREDICT_RIGHT_FIRST_END = 4,
    DF3_PREDICT_RIGHT_SECOND_END = 10,
    DF3_SYM_FIRST_END = 10,
    DF3_SYM_SECOND_END = 15
};
#endif
typedef struct {
    unsigned phase, row;
    df3Status_e status;
    float dt, threshold, gyro[3], nx[DF3_NX], dx[DF3_NE];
    float z[3], R[9], h[3], H[3 * DF3_NE];
    float L[9], y[3], HP[3 * DF3_NE], K[3 * DF3_NE];
    uint8_t columns[3][DF3_NE], count[3];
#ifdef USE_DF3_MULTIRATE
    /* A validated physical-slot hint, never a retained history pointer. */
    uint16_t historyHint;
    df3Rotation_t rotation;
    const df3GyroHistory_t *history;
    uint64_t cursorUs, endUs;
#endif
} df3CoreJob_t;

#ifdef USE_DF3_MULTIRATE
void df3CoreStartHistoryPredict(df3CoreJob_t *job, const df3GyroHistory_t *history, uint64_t startUs, uint64_t endUs);
#endif
void df3CoreStartPredict(df3CoreJob_t *job, const float gyro[3], float dt);
void df3CoreStartUpdate(df3CoreJob_t *job, const float z[3], const float R[9], const float h[3],
                        const float H[3 * DF3_NE], float threshold);
/* One bounded phase. true means complete; status includes rejection/error.
 * Core state/covariance are untouched until the final commit phase. */
bool df3CoreResume(df3CoreJob_t *job, df3Core_t *core, df3Workspace_t *work);

typedef struct {
    df3CoreJob_t coreJob;
#ifdef USE_DF3_BLACKBOX
    // Caller-owned observations. Never retain a pointer into EKF scratch.
    df3FusionTrace_t *trace;
#endif
#ifdef USE_DF3_MULTIRATE
    df3Event_t batch[DF3_EPOCH_CAPACITY];
    const df3GyroHistory_t *history;
    df3Rotation_t windowRotation;
    df3FlowQuantizedJob_t quantized;
    df3Estimate_t projected;
    uint64_t windowStartUs, windowCursorUs, windowEndUs, projectionTargetUs;
    float windowForce[3], windowM[9], windowBiasH[9];
    uint32_t epochs, historyFaults, burstFaults, stationaryUpdates;
    bool stationaryDue;
    uint16_t historyHint;
#else
    df3Event_t batch[DF3_EVENT_CAPACITY];
#endif
    df3Estimate_t committed;
    float committedGyro[3];
    uint64_t committedImuUs, committedAttitudeUs, committedVerticalUs;
    uint64_t horizonUs, startUs, lastNowUs, predictionUs;
    unsigned phase, index, count, operation, priorAxis, afterCore;
    float h[3], H[3 * DF3_NE], R[9], prior[3], z[3];
    uint32_t transactions, slices, cancelled;
    bool busy, published;
#ifdef USE_DF3_MULTIRATE
    /* Foreground output polling must not evict the worker replay hint. */
    uint16_t outputHistoryHint;
    /* Controller-facing translation observer. The committed cache above stays
     * an unmodified EKF projection; this state never feeds back into fusion.
     * P/V plus the preceding physical acceleration for motion prediction. */
    float outputMotion[9];
    uint64_t outputTimeUs;
#endif
} df3FusionJob_t;
#ifdef USE_DF3_MULTIRATE
/* Baseline ARM owner 4064 B, minus 56*40 B batch entries, plus the explicitly
 * allocated 1024 B frontend/scratch allowance, plus 32 B for the IMU
 * roughness observer and 48 B for the translation output observer (including
 * alignment). No second covariance or heap. */
_Static_assert(sizeof(df3FusionJob_t) + sizeof(df3ImuReducer_t) <= 2928,
               "Multirate owner/frontend exceeded the bounded storage allowance");
#endif

void df3FusionReset(df3FusionJob_t *job);
bool df3FusionEnqueue(df3FusionJob_t *job, df3Estimator_t *e, const df3Event_t *event);
bool df3FusionStart(df3FusionJob_t *job, df3Estimator_t *e, uint64_t nowUs);
#ifdef USE_DF3_MULTIRATE
/* Pure readiness: starting an epoch remains measured, bounded worker work. */
bool df3FusionDue(const df3FusionJob_t *job, const df3Estimator_t *e, uint64_t nowUs);
#endif
/* One phase. A scheduler callback may execute a small fixed phase budget;
 * never drain the complete transaction in an unbounded loop. */
void df3FusionResume(df3FusionJob_t *job, df3Estimator_t *e);
#ifdef USE_DF3_BUDGETED_WORKER
/* Stable scheduling cost bucket; never changes estimator math or state. */
unsigned df3FusionPhaseKey(const df3FusionJob_t *job);
#endif
/* Fast nominal projection with gradual P/V corrections for the controller.
 * Acceleration, attitude and biases retain their EKF meaning. Never reads a
 * partly updated covariance. Expiration checks use fused sensor timestamps;
 * covarianceTimeUs describes the raw fusion, not an observer covariance. */
bool df3FusionOutput(df3FusionJob_t *job, const df3Estimator_t *e, uint64_t nowUs, df3Estimate_t *out);
#endif
