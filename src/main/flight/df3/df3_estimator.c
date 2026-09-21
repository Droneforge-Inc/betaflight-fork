/* Bounded fixed-lag fusion with nominal output propagated to controller time.
 * Core model follows the SDK; DF3 has its own sensor-noise tuning. Bounded bias
 * covariance adaptation below uses persistent, range-anchored IMU innovation.
 * The new lag is scheduling for genuinely timestamped observations; it is not
 * a guessed correction applied to telemetry receipt timestamps. */
#include "df3_estimator.h"
#include "df3_flow.h"
#include "df3_profile.h"
#include <math.h>
#include <string.h>

static const float accelerationR[9] = {.324f, 0, 0, 0, .324f, 0, 0, 0, 1.2f};
static const float zeroAccelerationR[9] = {.04f, 0, 0, 0, .04f, 0, 0, 0, .04f};
static const float zeroVelocityR[9] = {.0025f, 0, 0, 0, .0025f, 0, 0, 0, .0025f};
static const float attitudeR[9] = {.00274155677808038f, 0, 0, 0, .00274155677808038f, 0, 0, 0, .0109662271123215f};

/* Persistent signed IMU innovation is evidence for changing bias; alternating
 * vibration and short maneuver residuals should not keep the bias gain high.
 * Use the existing bias state/cross-covariances. Sensor-noise weighting is
 * handled separately in the measurement covariance. The added covariance is nonnegative, preserving PSD.
 * Validate these bounds against physical motion as well as state error. */
static void trackAccelerometerBias(df3Estimator_t *e, uint64_t us, float residual)
{
    const float dt = e->accelBiasInnovationUs && us > e->accelBiasInnovationUs
                         ? fminf(.1f, (float)(us - e->accelBiasInnovationUs) * 1e-6f)
                         : 0;
    e->accelBiasInnovationUs = us;
    if (!e->dynamicsActive || us < e->rangeUs || us - e->rangeUs > 100000 ||
        !e->lastRangeDecision.surfaceLocked) {
        e->accelBiasInnovationMean = 0;
        return;
    }
    // 200 ms signed average; bound one bad sample's influence to 3 m/s^2.
    residual = fminf(3.f, fmaxf(-3.f, residual));
    e->accelBiasInnovationMean += dt / (.2f + dt) * (residual - e->accelBiasInnovationMean);
    // Leave nominal bias noise unchanged within a .15 m/s^2 deadband. Above
    // it, smoothly add at most 9 (m/s^2)^2 per second to body-Z bias variance.
    const float weight = fminf(1.f, fmaxf(0.f, (fabsf(e->accelBiasInnovationMean) - .15f) / .3f));
    const unsigned zBias = DF3_EBA + 2;
    e->core.P[zBias * DF3_NE + zBias] += 9.f * weight * dt;
}

void df3EstimatorReset(df3Estimator_t *e, uint32_t lagUs)
{
    memset(e, 0, sizeof(*e));
    e->lagUs = lagUs;
    e->flowHeightPerGain = 1;
    df3CoreReset(&e->core);
}

bool df3EstimatorInitialize(df3Estimator_t *e, uint64_t us, float range, const float q[4], const float gyro[3],
                            float strength)
{
    DF3_PROFILE_SCOPE(DF3_PROF_INITIALIZE);
    if (!e || !q || !gyro || e->dynamicsActive || !isfinite(range) || range >= -.001f || range <= -20) {
        return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(gyro[i])) {
            return false;
        }
    }
    const uint32_t lag = e->lagUs;
    df3EstimatorReset(e, lag);
    const float p[3] = {0, 0, range}, zero[3] = {0};
    if (!df3CoreInitialize(&e->core, p, zero, zero, q)) {
        return false;
    }
    df3RangeReset(&e->terrain, range, us, strength);
    memcpy(e->gyro, gyro, sizeof(e->gyro));
    e->fusionUs = e->bootstrapUs = e->imuUs = e->attitudeUs = e->verticalReferenceUs = e->rangeUs = us;
    for (unsigned i = 0; i < DF3_EVENT_KIND_COUNT; ++i) {
        e->newest[i] = us;
    }
    e->lastRange = range;
    e->lastStrength = strength;
    e->verticalUpdates = 1;
    e->initialized = true;
    return true;
}

bool df3EstimatorEnqueue(df3Estimator_t *e, const df3Event_t *event)
{
    DF3_PROFILE_SCOPE(DF3_PROF_ENQUEUE);
    if (!e || !event || !e->initialized || e->failed || (unsigned)event->kind >= DF3_EVENT_KIND_COUNT) {
        return false;
    }
    const unsigned fields[] = {6, 4, 2, 7, 0};
    for (unsigned i = 0; i < fields[event->kind]; ++i) {
        if (!isfinite(event->data[i])) {
            return false;
        }
    }
    if (event->us <= e->newest[event->kind] || event->us < e->fusionUs) {
        ++e->stale;
        return false;
    }
    if (e->count == DF3_EVENT_CAPACITY) {
        ++e->overflow;
        e->failed = true;
        return false;
    }
    unsigned index = e->count;
    while (index && (e->events[index - 1].us > event->us ||
                     (e->events[index - 1].us == event->us && e->events[index - 1].kind > event->kind))) {
        e->events[index] = e->events[index - 1];
        --index;
    }
    e->events[index] = *event;
    ++e->count;
    e->newest[event->kind] = event->us;
    return true;
}

static bool predictTo(df3Estimator_t *e, uint64_t us)
{
    if (us < e->fusionUs || us - e->fusionUs > 200000) {
        return false;
    }
    if (us == e->fusionUs) {
        return true;
    }
    DF3_PROFILE_SCOPE(DF3_PROF_PREDICT);
    const df3Status_e s = df3CorePredict(&e->core, &e->work, e->gyro, (float)(us - e->fusionUs) * 1e-6f);
    if (s != DF3_OK) {
        return false;
    }
    ++e->predictions;
    e->fusionUs = us;
    return true;
}

static bool update(df3Estimator_t *e, df3Observation_e kind, const float z[3], const float R[9], const float *q,
                   float threshold, const float *h, const float *H)
{
#ifdef USE_DF3_PROFILE
    const df3ProfileSection_e section = kind == DF3_ACCELERATION         ? DF3_PROF_ZERO_ACCEL
                                        : kind == DF3_VELOCITY           ? DF3_PROF_ZERO_VEL
                                        : kind == DF3_BODY_ACCELEROMETER ? DF3_PROF_ACCEL
                                        : kind == DF3_ATTITUDE           ? DF3_PROF_ATTITUDE
                                                                         : DF3_PROF_POSITION;
    DF3_PROFILE_SCOPE(section);
#endif
    // Robust-noise selection already linearizes the current state. Reuse
    // that exact observation when supplied, with the same core validation,
    // Joseph update and status handling. No state change may occur between
    // its construction and this call.
    const df3Status_e s = h ? df3CoreUpdateObservation(&e->core, &e->work, z, R, h, H, threshold, 0)
                            : df3CoreUpdate(&e->core, &e->work, kind, z, R, q, threshold, 0);
    if (s == DF3_OK) {
        ++e->updates;
    }
    if (s == DF3_REJECTED) {
        ++e->rejected;
    }
    return s == DF3_OK || s == DF3_REJECTED;
}

static bool apply(df3Estimator_t *e, const df3Event_t *event)
{
    DF3_PROFILE_SCOPE((df3ProfileSection_e)(DF3_PROF_EVENT_IMU + event->kind));
    const float zero[3] = {0};
    float *x = e->core.x;
    if (!predictTo(e, event->us)) {
        return false;
    }
    switch (event->kind) {
    case DF3_EVENT_IMU: {
        float h[3], H[54], prior[3] = {0}, R[9];
        if (!e->dynamicsActive && !update(e, DF3_ACCELERATION, zero, zeroAccelerationR, 0, 16.27f, 0, 0)) {
            return false;
        }
        if (!df3ModelObserve(x, DF3_BODY_ACCELEROMETER, 0, h, H)) {
            return false;
        }
        trackAccelerometerBias(e, event->us, event->data[5] - h[2]);
        for (unsigned axis = 0; axis < 3; ++axis) {
            for (unsigned i = 0; i < 18; ++i) {
                if (H[axis * 18 + i] == 0) {
                    continue;
                }
                for (unsigned j = 0; j < 18; ++j) {
                    prior[axis] += H[axis * 18 + i] * e->core.P[i * 18 + j] * H[axis * 18 + j];
                }
            }
        }
        df3RobustAccelerometerCovariance(event->data + 3, h, prior, accelerationR, R);
        if (!update(e, DF3_BODY_ACCELEROMETER, event->data + 3, R, 0, 16.27f, h, H)) {
            return false;
        }
        if (!e->dynamicsActive && !update(e, DF3_VELOCITY, zero, zeroVelocityR, 0, 16.27f, 0, 0)) {
            return false;
        }
        memcpy(e->gyro, event->data, sizeof(e->gyro));
        e->imuUs = event->us;
        break;
    }
    case DF3_EVENT_ATTITUDE: {
        float h[3], H[54], R[9], residual[3];
        if (!df3ModelObserve(x, DF3_ATTITUDE, event->data, h, H)) {
            return false;
        }
        for (unsigned i = 0; i < 3; ++i) {
            residual[i] = -h[i];
        }
        df3RobustAttitudeCovariance(residual, e->core.P, attitudeR, R);
        if (!update(e, DF3_ATTITUDE, zero, R, event->data, 16.27f, h, H)) {
            return false;
        }
        e->attitudeUs = event->us;
        break;
    }
    case DF3_EVENT_RANGE: {
        const float range = event->data[0], strength = event->data[1];
        if (range >= -.001f || range <= -20) {
            break;
        }
        const df3RangeDecision_t decision =
            df3RangeObserve(&e->terrain, range, event->us, x[2], e->dynamicsActive ? x[5] : 0, strength,
                            e->core.P[2 * 18 + 2], e->dynamicsActive ? e->core.P[5 * 18 + 5] : 0);
        e->lastRangeDecision = decision;
        e->lastRange = range;
        e->lastStrength = strength;
        e->rangeUs = event->us;
        if (!e->dynamicsActive) {
            if (decision.surfaceLocked) {
                x[2] = range;
                df3RangeReset(&e->terrain, range, event->us, strength);
                e->verticalReferenceUs = event->us;
                e->verticalUpdates = 3;
            }
        } else if (decision.hasPosition) {
            const float z[3] = {x[0], x[1], decision.position};
            const float R[9] = {1e6f, 0, 0, 0, 1e6f, 0, 0, 0, decision.positionVariance};
            if (!update(e, DF3_POSITION, z, R, 0, 0, 0, 0)) {
                return false;
            }
            if (decision.surfaceLocked) {
                e->verticalReferenceUs = event->us;
                if (e->verticalUpdates < 3) {
                    ++e->verticalUpdates;
                }
            }
        }
        break;
    }
    case DF3_EVENT_FLOW: {
        if (!e->dynamicsActive || event->us < e->imuUs || event->us - e->imuUs > 200000 || event->us < e->attitudeUs ||
            event->us - e->attitudeUs > 250000) {
            break;
        }
        float z[3] = {event->data[0], event->data[1], 0};
        float R[9], h[3], H[3 * DF3_NE], prior[2] = {0};
        if (!df3FlowObserveOffset(x, event->data[5], e->flowSensorOffset, h, H)) {
            break;
        }
        for (unsigned axis = 0; axis < 2; ++axis) {
            for (unsigned i = 0; i < DF3_NE; ++i) {
                if (H[axis * DF3_NE + i] == 0) {
                    continue;
                }
                for (unsigned j = 0; j < DF3_NE; ++j) {
                    prior[axis] += H[axis * DF3_NE + i] * e->core.P[i * DF3_NE + j] * H[axis * DF3_NE + j];
                }
            }
        }
        const df3FlowNoise_t noise = {
            .pRadps = event->data[2] - x[DF3_BG], .qRadps = event->data[3] - x[DF3_BG + 1],
            .quality = event->data[4], .height = event->data[5] * e->flowHeightPerGain,
            .rangeVariance = df3RangeMeasurementVariance(&e->terrain, event->data[6])};
        df3RobustFlowCovariance(z, h, prior, &noise, R);
        if (!df3FlowQuantizedObservation(e->flowQuantumPerGain * event->data[5], prior, z, h, H, R)) {
            break;
        }
        DF3_PROFILE_SCOPE(DF3_PROF_FLOW_UPDATE);
        const df3Status_e status = df3CoreUpdateObservation(&e->core, &e->work, z, R, h, H, 16.27f, 0);
        if (status == DF3_OK) {
            ++e->updates;
        } else if (status == DF3_REJECTED) {
            ++e->rejected;
        } else {
            return false;
        }
        break;
    }
    case DF3_EVENT_ACTIVATE:
        if (!e->dynamicsActive) {
            e->dynamicsActive = true;
            if (event->us >= e->rangeUs && event->us - e->rangeUs <= 300000) {
                df3RangeReset(&e->terrain, e->lastRange, e->rangeUs, e->lastStrength);
            }
        }
        break;
    default:
        return false;
    }
    return df3CoreHealthy(&e->core);
}

bool df3EstimatorTick(df3Estimator_t *e, uint64_t now, df3Estimate_t *out)
{
    if (!e || !out) {
        return false;
    }
    DF3_PROFILE_SCOPE(DF3_PROF_FUSION);
    DF3_PROFILE_GAUGE(DF3_PROF_QUEUE_IN, e->count);
    if (e->count && now >= e->events[0].us) {
        DF3_PROFILE_GAUGE(DF3_PROF_OLDEST_AGE_US, now - e->events[0].us);
    }
    memset(out, 0, sizeof(*out));
    out->timeUs = now;
    if (!e->initialized || e->failed || now < e->lastTickUs || now < e->bootstrapUs) {
        return false;
    }
    e->lastTickUs = now;
    const uint64_t horizon = now >= e->lagUs ? now - e->lagUs : 0;
    unsigned consumed = 0;
    while (consumed < e->count && e->events[consumed].us <= horizon) {
        if (!apply(e, &e->events[consumed])) {
            DF3_PROFILE_GAUGE(DF3_PROF_EVENTS_TICK, consumed + 1);
            e->failed = true;
            return false;
        }
        ++consumed;
    }
    DF3_PROFILE_GAUGE(DF3_PROF_EVENTS_TICK, consumed);
    if (consumed) {
        e->count -= consumed;
        memmove(e->events, e->events + consumed, e->count * sizeof(e->events[0]));
    }
    DF3_PROFILE_GAUGE(DF3_PROF_QUEUE_OUT, e->count);
    if (horizon > e->fusionUs && !predictTo(e, horizon)) {
        e->failed = true;
        return false;
    }
    DF3_PROFILE_GAUGE(DF3_PROF_FUSION_AGE_US, now - e->fusionUs);
    {
        DF3_PROFILE_SCOPE(DF3_PROF_OUTPUT);
        memcpy(out->x, e->core.x, sizeof(out->x));
        out->terrainDown = e->terrain.terrain;
        out->covarianceTimeUs = e->fusionUs;
        /* Only nominal state is needed at the controller rate. Covariance stays
     * on its explicitly reported fusion horizon, avoiding a dense replay per
     * publication. Each queued gyro owns only its following time interval. */
        uint64_t projected = e->fusionUs;
        float gyro[3];
        memcpy(gyro, e->gyro, sizeof(gyro));
        for (unsigned i = 0; i < e->count; ++i) {
            const df3Event_t *event = &e->events[i];
            if (event->us > now) {
                break;
            }
            if (event->kind != DF3_EVENT_IMU) {
                continue;
            }
            if (!df3ProjectNominal(out->x, gyro, (float)(event->us - projected) * 1e-6f)) {
                return false;
            }
            projected = event->us;
            memcpy(gyro, event->data, sizeof(gyro));
        }
        if (now < projected || now - projected > 200000 ||
            !df3ProjectNominal(out->x, gyro, (float)(now - projected) * 1e-6f)) {
            return false;
        }
        out->valid = e->verticalUpdates >= 3 && now - e->bootstrapUs >= 150000 && now >= e->imuUs &&
                     now - e->imuUs <= 200000 && now >= e->attitudeUs && now - e->attitudeUs <= 250000 &&
                     df3CoreHealthy(&e->core);
        out->verticalReferenceValid =
            out->valid && now >= e->verticalReferenceUs && now - e->verticalReferenceUs <= 500000;
        return out->valid;
    }
}

#ifdef USE_DF3_RESUMABLE
#include "df3_resumable_estimator.inc"
#endif
