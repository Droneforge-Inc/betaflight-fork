/* Same range classification thresholds as the SDK DfEsekfMeasurementPolicy.
 * Accepted range position weight is tuned separately for onboard DF3.
 * Terrain is outside the 19-state Rednose model; range updates global position
 * OR terrain for a sample, never both. See tools/df3/test_policy.py. */
#include "df3_policy.h"
#include <math.h>
#include <string.h>

static float clamp(float v, float lo, float hi)
{
    return fminf(hi, fmaxf(lo, v));
}
static float qualityNormalized(float q)
{
    return isfinite(q) ? clamp(q / 255, 0, 1) : 0;
}
static float strengthScale(float strength)
{
    return 1 + 4 * (1 - qualityNormalized(strength));
}
float df3RangeMeasurementVariance(const df3RangePolicy_t *p, float strength)
{
    return (.001f + 5.f * p->measuredNoiseVariance) * strengthScale(strength);
}
static float rangeRateVariance(float dt)
{
    const float s = fmaxf(1, .08f / dt);
    return .04f * s * s;
}
static void storeRange(df3RangePolicy_t *p, float range, uint64_t us, float scale)
{
    p->previousRange = range;
    p->previousUs = us;
    p->previousStrengthScale = scale;
}

void df3RangeReset(df3RangePolicy_t *p, float initialRange, uint64_t us, float strength)
{
    memset(p, 0, sizeof(*p));
    storeRange(p, initialRange, us, strengthScale(strength));
}

/* Sensor-noise proxy: the third divided difference
 * eliminates constant position, velocity and acceleration, including unequal
 * sample spacing. Divide by the coefficient energy to retain m^2 units. */
static void observeRangeNoise(df3RangePolicy_t *p, float range, uint64_t us)
{
    if (p->noiseCount && (us <= p->noiseUs[p->noiseCount - 1] ||
                         us - p->noiseUs[p->noiseCount - 1] > 250000)) {
        p->noiseCount = 0;
    }
    if (p->noiseCount == 3) {
        const float values[4] = {p->noiseRange[0], p->noiseRange[1], p->noiseRange[2], range};
        const uint64_t times[4] = {p->noiseUs[0], p->noiseUs[1], p->noiseUs[2], us};
        float residual = 0, weightEnergy = 0;
        for (unsigned i = 0; i < 4; ++i) {
            float coefficient = 1;
            for (unsigned j = 0; j < 4; ++j) {
                if (j != i) {
                    coefficient /= (float)((int64_t)times[i] - (int64_t)times[j]) * .00005f;
                }
            }
            residual += coefficient * values[i];
            weightEnergy += coefficient * coefficient;
        }
        const float variance = fminf(1.f, residual * residual / weightEnergy);
        const float dt = (float)(us - p->noiseUs[2]) * 1e-6f;
        p->measuredNoiseVariance += dt / (.1f + dt) * (variance - p->measuredNoiseVariance);
        for (unsigned i = 0; i < 2; ++i) {
            p->noiseRange[i] = p->noiseRange[i + 1];
            p->noiseUs[i] = p->noiseUs[i + 1];
        }
        p->noiseCount = 2;
    }
    p->noiseRange[p->noiseCount] = range;
    p->noiseUs[p->noiseCount++] = us;
}

df3RangeDecision_t df3RangeObserve(df3RangePolicy_t *p, float range, uint64_t us, float position, float velocity,
                                   float strength, float positionVar, float velocityVar)
{
    df3RangeDecision_t d = {.mode = DF3_RANGE_GLOBAL,
                            .positionVariance = .01f,
                            .velocityVariance = .25f,
                            .terrain = p->terrain,
                            .surfaceLocked = true};
    if (!isfinite(range) || !isfinite(position) || !isfinite(velocity)) {
        return d;
    }
    observeRangeNoise(p, range, us);
    const float scale = strengthScale(strength);
    const bool hasDelta = p->previousUs != 0 && us > p->previousUs;
    const uint64_t deltaUs = hasDelta ? us - p->previousUs : 0;
    const bool hasDt = hasDelta && deltaUs >= 20000 && deltaUs <= 250000;
    const float dt = hasDt ? (float)deltaUs * 1e-6f : 0;
    const float delta = range - p->previousRange;
    const float rate = hasDt ? delta / dt : 0;
    d.positionInnovation = range - (position - p->terrain);
    if (hasDt) {
        d.velocityResidual = rate - velocity;
    }
    const float innovationVariance = fmaxf(0, positionVar) + .01f;
    const bool inlier = fabsf(d.positionInnovation) <= 2 * sqrtf(innovationVariance);
    const float stepGate = 2 * sqrtf(.02f), cumulativeGate = 3 * sqrtf(.02f);
    const bool stepOutlier = hasDelta && fabsf(delta) > stepGate;
    bool change = !inlier && stepOutlier;
    const bool direct = change;
    float terrainDelta = delta;
    bool pending = false;
    const bool inWindow = p->lastTerrainUs != 0 && us > p->lastTerrainUs && us - p->lastTerrainUs <= 1750000;
    const bool significant = fabsf(delta) >= .03f;
    float continuationNis = 0;
    if (hasDt) {
        const float vv = fmaxf(0, velocityVar) + rangeRateVariance(dt);
        continuationNis = d.positionInnovation * d.positionInnovation / innovationVariance +
                          d.velocityResidual * d.velocityResidual / vv;
    }
    if (change) {
        if (p->continuationDelta != 0 && delta * p->continuationDelta > 0) {
            terrainDelta += p->continuationDelta;
        }
        p->continuationDelta = 0;
    } else if (!inWindow) {
        if (fabsf(p->continuationDelta) > cumulativeGate) {
            change = true;
            terrainDelta = p->continuationDelta;
            if (hasDt && significant && delta * terrainDelta > 0) {
                terrainDelta += delta;
            }
        }
        p->continuationDelta = 0;
        p->continuationEligible = false;
    } else if (hasDt) {
        if (p->continuationDelta != 0) {
            if (!significant) {
                if (fabsf(p->continuationDelta) > cumulativeGate) {
                    change = true;
                    terrainDelta = p->continuationDelta;
                }
                p->continuationDelta = 0;
            } else if (delta * p->continuationDelta > 0) {
                const bool evidence = d.positionInnovation * p->continuationDelta > 0 &&
                                      d.velocityResidual * p->continuationDelta > 0 &&
                                      fabsf(d.positionInnovation) >= .04f && continuationNis > 1;
                if (evidence) {
                    p->continuationDelta += delta;
                    pending = true;
                } else {
                    if (fabsf(p->continuationDelta) > cumulativeGate) {
                        change = true;
                        terrainDelta = p->continuationDelta;
                    }
                    p->continuationDelta = 0;
                }
            } else {
                if (fabsf(p->continuationDelta) > cumulativeGate) {
                    change = true;
                    terrainDelta = p->continuationDelta;
                }
                p->continuationDelta = 0;
            }
        } else if (p->continuationEligible && significant && delta * p->lastTerrainDelta < 0 &&
                   d.positionInnovation * p->lastTerrainDelta < 0 && d.velocityResidual * p->lastTerrainDelta < 0 &&
                   fabsf(d.positionInnovation) >= .04f && continuationNis > 1) {
            p->continuationDelta = delta;
            p->continuationEligible = false;
            pending = true;
        }
    }
    d.plausible = !change && !pending;
    if (change) {
        if (!direct) {
            terrainDelta *= .70f;
        }
        const float terrain = p->terrain - terrainDelta;
        d.mode = terrain < p->terrain ? DF3_RANGE_ABSORB : DF3_RANGE_EXPOSE;
        p->terrain = terrain;
        p->lastTerrainUs = us;
        p->lastTerrainDelta = terrainDelta;
        p->continuationDelta = 0;
        p->continuationEligible = direct;
        d.terrain = terrain;
        d.surfaceLocked = false;
        storeRange(p, range, us, scale);
        return d;
    }
    if (pending) {
        d.surfaceLocked = false;
        storeRange(p, range, us, scale);
        return d;
    }
    d.hasPosition = true;
    d.position = range + p->terrain;
    /* Constrain false vertical motion from changing acceleration bias.
     * The variance floor and measured-noise multiplier are fusion tuning,
     * not sensor accuracy claims.
     * Keep terrain gates above unchanged and still weaken poor returns.
     * Bias response and physical motion must be checked together. */
    d.positionVariance = df3RangeMeasurementVariance(p, strength);
    if (hasDt && fabsf(rate) <= 13.696244240f) {
        d.hasVelocity = true;
        d.velocity = rate;
        d.velocityVariance = rangeRateVariance(dt) * fmaxf(p->previousStrengthScale, scale);
    }
    storeRange(p, range, us, scale);
    return d;
}

static float huber(float residual, float prior, float base, float sigma)
{
    const float variance = fmaxf(1e-9f, fmaxf(0, prior) + base), nis = residual * residual / variance;
    if (!isfinite(nis) || nis <= sigma * sigma) {
        return base;
    }
    const float weight = sigma / sqrtf(nis);
    return base / (weight * weight);
}

void df3RobustVelocityCovariance(const float z[3], const float pred[3], const float P[324], float p, float q,
                                 float quality, float R[9])
{
    const float prior[2] = {P[DF3_EV * 18 + DF3_EV], P[(DF3_EV + 1) * 18 + DF3_EV + 1]};
    const df3FlowNoise_t noise = {.pRadps = p, .qRadps = q, .quality = quality, .height = 1};
    df3RobustFlowCovariance(z, pred, prior, &noise, R);
}

void df3RobustFlowCovariance(const float z[3], const float pred[3], const float prior[2],
                             const df3FlowNoise_t *noise, float R[9])
{
    const float r2 = noise->pRadps * noise->pRadps + noise->qRadps * noise->qRadps;
    const float rate = sqrtf(r2), base = 1 + 2 * r2;
    const float low = .3490658503988659f, high = 1.3962634015954636f;
    const float ratio = clamp((rate - low) / (high - low), 0, 1);
    const float rotationScale = rate <= low ? base : base + ratio * ratio * (fmaxf(base, 25) - base);
    const float error = 1 - qualityNormalized(noise->quality);
    const float height2 = noise->height * noise->height;
    /* v = height * angular flow. Preserve the established near-ground floor;
     * above 1 m the angular/rotation noise grows with squared distance.
     * Linearize range scaling at predicted velocity; measurement residuals
     * are handled separately by the robust weighting below.
     * One shared range error correlates X/Y. Its trace times identity bounds
     * that rank-one covariance while retaining the quantizer's diagonal R.
     * Gyro-bias state uncertainty is already in HPH', not added again here. */
    const float rangeNoise = noise->rangeVariance * (pred[0] * pred[0] + pred[1] * pred[1]) / fmaxf(height2, .0001f);
    const float variance = .135f * rotationScale * (1 + 4 * error * error) * fmaxf(1, height2) + rangeNoise;
    memset(R, 0, 9 * sizeof(float));
    for (unsigned i = 0; i < 2; ++i) {
        R[4 * i] = huber(z[i] - pred[i], fmaxf(0, prior[i]), variance, 2.5f);
    }
    R[8] = 1e6f;
}

void df3RobustAccelerometerCovariance(const float z[3], const float pred[3], const float prior[3], const float base[9],
                                      float R[9])
{
    memset(R, 0, 9 * sizeof(float));
    for (unsigned i = 0; i < 3; ++i) {
        R[4 * i] = huber(z[i] - pred[i], fmaxf(0, prior[i]), base[4 * i], 3);
    }
}

void df3RobustAttitudeCovariance(const float residual[3], const float P[324], const float base[9], float R[9])
{
    memset(R, 0, 9 * sizeof(float));
    for (unsigned i = 0; i < 3; ++i) {
        R[4 * i] = huber(residual[i], P[(DF3_ET + i) * 18 + DF3_ET + i], base[4 * i], 2.5f);
    }
}
