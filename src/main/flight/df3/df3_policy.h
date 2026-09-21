/* Droneforge DF3 port of DfEsekfMeasurementPolicy. */
#pragma once
#include "df3_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { DF3_RANGE_GLOBAL, DF3_RANGE_ABSORB, DF3_RANGE_EXPOSE } df3RangeMode_e;
typedef struct {
    float terrain, previousRange, lastTerrainDelta, continuationDelta, previousStrengthScale;
    uint64_t previousUs, lastTerrainUs;
    bool continuationEligible;
    float noiseRange[3], measuredNoiseVariance;
    uint64_t noiseUs[3];
    unsigned noiseCount;
} df3RangePolicy_t;
typedef struct {
    df3RangeMode_e mode;
    bool hasPosition, hasVelocity, plausible, surfaceLocked;
    float position, velocity, positionVariance, velocityVariance;
    float terrain, positionInnovation, velocityResidual;
} df3RangeDecision_t;
typedef struct {
    float pRadps, qRadps, quality;
    float height, rangeVariance; // metres, m^2; independent range measurement uncertainty
} df3FlowNoise_t;

void df3RangeReset(df3RangePolicy_t *policy, float initialRangeDown, uint64_t timeUs, float strength);
float df3RangeMeasurementVariance(const df3RangePolicy_t *policy, float strength);
df3RangeDecision_t df3RangeObserve(df3RangePolicy_t *policy, float rangeDown, uint64_t timeUs, float positionDown,
                                   float velocityDown, float strength, float positionVariance, float velocityVariance);
void df3RobustVelocityCovariance(const float measurement[3], const float prediction[3], const float P[324],
                                 float pRadps, float qRadps, float quality, float R[9]);
void df3RobustFlowCovariance(const float measurement[3], const float prediction[3], const float priorVariance[2],
                             const df3FlowNoise_t *noise, float R[9]);
void df3RobustAccelerometerCovariance(const float measurement[3], const float prediction[3],
                                      const float priorVariance[3], const float base[9], float R[9]);
void df3RobustAttitudeCovariance(const float residual[3], const float P[324], const float base[9], float R[9]);

#ifdef __cplusplus
}
#endif
