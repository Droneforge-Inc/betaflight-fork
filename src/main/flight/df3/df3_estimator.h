/* Portable onboard runtime. Sensor adapters supply FC-clock acquisition times. */
#pragma once
#include "df3_core.h"
#include "df3_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { DF3_EVENT_CAPACITY = 64 };
// Time constant of the controller-facing position/velocity correction observer.
#define DF3_OUTPUT_OBSERVER_TAU_S .015f
typedef enum {
    DF3_EVENT_IMU,
    DF3_EVENT_ATTITUDE,
    DF3_EVENT_RANGE,
    DF3_EVENT_FLOW,
    DF3_EVENT_ACTIVATE,
    DF3_EVENT_KIND_COUNT
} df3EventKind_e;
typedef struct {
    uint64_t us;
    df3EventKind_e kind;
    /* IMU: gyro FRD rad/s, accelerometer FRD specific force m/s2.
     * ATTITUDE: q_LB wxyz. RANGE: negative clearance, strength.
     * FLOW: zero-bias compensated body forward/right m/s, interval-average
     * FRD p/q, quality, height*rotationScale, same-report range strength.
     * Bias is applied at fusion time.
     * ACTIVATE: none. */
    float data[7];
} df3Event_t;
typedef struct {
    df3Core_t core;
    df3Workspace_t work;
    df3RangePolicy_t terrain;
    df3Event_t events[DF3_EVENT_CAPACITY];
    uint64_t newest[DF3_EVENT_KIND_COUNT], fusionUs, bootstrapUs, lastTickUs;
    uint64_t imuUs, attitudeUs, verticalReferenceUs, rangeUs;
    float gyro[3], lastRange, lastStrength;
    /* Sensor metadata: normalized m/s quantum divided by rotationScale.
     * Multiply by the FLOW event's height*rotationScale for body m/s bins.
     * Zero retains legacy continuous observations. Set after initialization. */
    float flowQuantumPerGain;
    float flowHeightPerGain; // 1 / calibrated rotationScale; default 1
    /* Fixed calibrated lens offset, body FRD metres. Set after initialization.
     * Do not include this in flowQuantumPerGain: a lever arm changes gyro
     * sensitivity but does not change the sensor's velocity bin width. */
    float flowSensorOffset[3];
    uint32_t lagUs, predictions, updates, rejected, stale, overflow;
    uint16_t count;
    uint8_t verticalUpdates;
    bool initialized, failed, dynamicsActive;
#ifdef USE_DF3_MULTIRATE
    bool stationary;
#endif
    df3RangeDecision_t lastRangeDecision;
} df3Estimator_t;
typedef struct {
    float x[DF3_NX];
    float terrainDown;
    uint64_t timeUs, covarianceTimeUs;
    bool valid, verticalReferenceValid;
} df3Estimate_t;

void df3EstimatorReset(df3Estimator_t *estimator, uint32_t lagUs);
bool df3EstimatorInitialize(df3Estimator_t *estimator, uint64_t us, float negativeClearance, const float q[4],
                            const float gyro[3], float strength);
bool df3EstimatorEnqueue(df3Estimator_t *estimator, const df3Event_t *event);
bool df3EstimatorTick(df3Estimator_t *estimator, uint64_t nowUs, df3Estimate_t *out);

#ifdef __cplusplus
}
#endif
