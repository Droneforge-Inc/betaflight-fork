/* Production: fusion transaction owner, bounded work and committed estimates.
 * Included by df3_estimator.c to share its private observation constants and
 * helpers. Production uses the USE_DF3_MULTIRATE branches for windowed sensor
 * fusion and gyro-history prediction. Their non-multirate alternatives are
 * retained research baselines for standalone comparisons. */
#include "df3_resumable.h"

enum {
    FUSION_SELECT = 1,
    FUSION_CORE,
    FUSION_EVENT,
    FUSION_IMU_OBSERVE,
    FUSION_PRIOR,
    FUSION_IMU_UPDATE,
    FUSION_IMU_ZERO_VEL,
    FUSION_FLOW_UPDATE,
    FUSION_RANGE_DONE,
    FUSION_EVENT_DONE,
    FUSION_PUBLISH
#ifdef USE_DF3_MULTIRATE
    ,
    FUSION_WINDOW_BACK,
    FUSION_WINDOW_FORWARD,
    FUSION_WINDOW_FINISH,
    FUSION_FLOW_QUANTIZED,
    FUSION_PROJECT
#endif
};

void df3FusionReset(df3FusionJob_t *j)
{
    memset(j, 0, sizeof(*j));
#ifdef USE_DF3_MULTIRATE
    j->historyHint = UINT16_MAX;
    j->outputHistoryHint = UINT16_MAX;
#endif
}

#ifdef USE_DF3_BUDGETED_WORKER
unsigned df3FusionPhaseKey(const df3FusionJob_t *j)
{
#ifdef USE_DF3_MULTIRATE
    if (!j->busy) {
        return 0; // Bounded epoch start, independently measured.
    }
    if (j->phase == FUSION_PROJECT) {
        return 79;
    }
#endif
    if (j->phase != FUSION_CORE) {
        return j->phase;
    }
    const unsigned row = j->coreJob.row;
    const unsigned rowGroup =
#ifdef USE_DF3_MULTIRATE
        j->coreJob.phase == 2    ? (row < DF3_PREDICT_LEFT_FIRST_END    ? 0
                                    : row < DF3_PREDICT_LEFT_SECOND_END ? 1
                                                                        : 2)
        : j->coreJob.phase == 3  ? (row < DF3_PREDICT_RIGHT_FIRST_END    ? 0
                                    : row < DF3_PREDICT_RIGHT_SECOND_END ? 1
                                                                         : 2)
        : j->coreJob.phase == 17 ? (row < DF3_SYM_FIRST_END    ? 0
                                    : row < DF3_SYM_SECOND_END ? 1
                                                               : 2)
                                 :
#endif
                                 row / DF3_SLICE_ROWS;
    return 16 + 3 * j->coreJob.phase + (rowGroup < 3 ? rowGroup : 2);
}
#endif

bool df3FusionEnqueue(df3FusionJob_t *j, df3Estimator_t *e, const df3Event_t *event)
{
    if (!j || !e || !event || !e->initialized || e->failed || (unsigned)event->kind >= DF3_EVENT_KIND_COUNT) {
        return false;
    }
    const unsigned fields[] = {6, 4, 2, 7, 0};
    for (unsigned i = 0; i < fields[event->kind]; ++i) {
        if (!isfinite(event->data[i])) {
            return false;
        }
    }
    /* Reserve the in-flight horizon. An instantaneous baseline would already
   * have fused up to this time. Equal-time late events go in the NEXT batch,
   * preserving receipt order across the transaction boundary. */
    if (event->us <= e->newest[event->kind] || event->us < e->fusionUs || (j->busy && event->us < j->horizonUs)) {
        ++e->stale;
        return false;
    }
    const unsigned remaining = j->busy ? j->count - j->index : 0;
    if (e->count + remaining >= DF3_EVENT_CAPACITY) {
        ++e->overflow;
        e->failed = true;
        return false;
    }
    return df3EstimatorEnqueue(e, event);
}

static void fusionSnapshot(df3Estimate_t *snapshot, const df3Estimator_t *e)
{
    memset(snapshot, 0, sizeof(*snapshot));
    memcpy(snapshot->x, e->core.x, sizeof(snapshot->x));
    snapshot->covarianceTimeUs = e->fusionUs;
    snapshot->timeUs = e->fusionUs;
    snapshot->terrainDown = e->terrain.terrain;
    snapshot->valid = e->verticalUpdates >= 3 && df3CoreHealthy(&e->core);
}

static void fusionPublishMetadata(df3FusionJob_t *j, const df3Estimator_t *e)
{
    memcpy(j->committedGyro, e->gyro, sizeof(j->committedGyro));
    j->committedImuUs = e->imuUs;
    j->committedAttitudeUs = e->attitudeUs;
    j->committedVerticalUs = e->verticalReferenceUs;
    j->published = true;
}

static void fusionPublish(df3FusionJob_t *j, const df3Estimator_t *e)
{
    fusionSnapshot(&j->committed, e);
    fusionPublishMetadata(j, e);
}

#ifdef USE_DF3_MULTIRATE
/* Internal event-array transfer: the arrays are disjoint (batch copy), or the
 * destination precedes the source (queue compaction). Forward blocks therefore
 * preserve overlap semantics. Event alignment/size allow whole-word transfers,
 * including padding, with no float conversions or event reordering. The G4's
 * size-optimized libc memmove uses a byte loop; keep this local loop from being
 * lowered back to that call. No extra buffer or scheduler yield is introduced. */
typedef uint32_t df3EventCopyWord_t __attribute__((may_alias));
_Static_assert(_Alignof(df3Event_t) >= _Alignof(df3EventCopyWord_t) &&
                   sizeof(df3Event_t) % sizeof(df3EventCopyWord_t) == 0,
               "DF3 event transfers require whole aligned words");

__attribute__((noipa, optimize("O2", "no-tree-loop-distribute-patterns", "no-tree-vectorize", "no-if-conversion",
                               "no-if-conversion2"))) static void
df3CopyEventsForward(df3Event_t *destination, const df3Event_t *source, unsigned count)
{
    df3EventCopyWord_t *dst = (df3EventCopyWord_t *)destination;
    const df3EventCopyWord_t *src = (const df3EventCopyWord_t *)source;
    unsigned words = count * (sizeof(df3Event_t) / sizeof(df3EventCopyWord_t));
    for (; words >= 4; words -= 4, dst += 4, src += 4) {
        const uint32_t a = src[0], b = src[1], c = src[2], d = src[3];
        dst[0] = a;
        dst[1] = b;
        dst[2] = c;
        dst[3] = d;
    }
    while (words--) {
        *dst++ = *src++;
    }
}

bool df3FusionDue(const df3FusionJob_t *j, const df3Estimator_t *e, uint64_t now)
{
    if (!j || !e || j->busy || !j->history || !e->initialized || e->failed || now < e->lastTickUs ||
        now < e->bootstrapUs || now < e->lagUs) {
        return false;
    }
    const uint64_t next = (e->fusionUs / DF3_EPOCH_US + 1) * DF3_EPOCH_US;
    return next <= now - e->lagUs;
}
#endif

bool df3FusionStart(df3FusionJob_t *j, df3Estimator_t *e, uint64_t now)
{
    if (!j || !e || j->busy || !e->initialized || e->failed || now < e->lastTickUs || now < e->bootstrapUs) {
        return false;
    }
    if (!j->published) {
        fusionPublish(j, e);
    }
    j->startUs = j->lastNowUs = e->lastTickUs = now;
#ifdef USE_DF3_MULTIRATE
    if (!j->history) {
        return false;
    }
    const uint64_t next = (e->fusionUs / DF3_EPOCH_US + 1) * DF3_EPOCH_US;
    if (now < e->lagUs || next > now - e->lagUs) {
        return false;
    }
    j->horizonUs = next;
#else
    /* Research: the pre-multirate owner advances directly to the lag horizon. */
    j->horizonUs = now >= e->lagUs ? now - e->lagUs : 0;
#endif
    j->count = 0;
    j->index = 0;
#ifdef USE_DF3_MULTIRATE
    /* Half-open epochs [start,end): a boundary observation belongs to the next
   * transaction, retaining its real timestamp and deterministic type order. */
    unsigned kinds[DF3_EVENT_KIND_COUNT] = {0};
    const unsigned maximum[DF3_EVENT_KIND_COUNT] = {1, 1, DF3_EPOCH_MTF_MAX, DF3_EPOCH_MTF_MAX, 1};
    while (j->count < e->count && e->events[j->count].us < j->horizonUs) {
        if (j->count == DF3_EPOCH_CAPACITY || ++kinds[e->events[j->count].kind] > maximum[e->events[j->count].kind]) {
            ++j->burstFaults;
            e->failed = true;
            return false;
        }
        ++j->count;
    }
#else
    while (j->count < e->count && e->events[j->count].us <= j->horizonUs) {
        ++j->count;
    }
#endif
#ifdef USE_DF3_MULTIRATE
    if (j->count) {
        df3CopyEventsForward(j->batch, e->events, j->count);
        e->count -= j->count;
        if (e->count) {
            df3CopyEventsForward(e->events, e->events + j->count, e->count);
        }
    }
#else
    memcpy(j->batch, e->events, j->count * sizeof(e->events[0]));
    e->count -= j->count;
    memmove(e->events, e->events + j->count, e->count * sizeof(e->events[0]));
#endif
    j->phase = FUSION_SELECT;
    j->busy = true;
    return true;
}

static void fusionFail(df3FusionJob_t *j, df3Estimator_t *e)
{
    e->failed = true;
    j->busy = false;
    j->committed.valid = j->committed.verticalReferenceValid = false;
}

static void fusionPredict(df3FusionJob_t *j, df3Estimator_t *e, uint64_t us, unsigned after)
{
    if (us < e->fusionUs || us - e->fusionUs > 200000) {
        fusionFail(j, e);
        return;
    }
    if (us == e->fusionUs) {
        j->phase = after;
        return;
    }
    j->predictionUs = us;
    j->operation = 0;
    j->afterCore = after;
#ifdef USE_DF3_MULTIRATE
    df3CoreStartHistoryPredict(&j->coreJob, j->history, e->fusionUs, us);
#else
    /* Research: predict with one held gyro value instead of measured history. */
    df3CoreStartPredict(&j->coreJob, e->gyro, (float)(us - e->fusionUs) * 1e-6f);
#endif
    j->phase = FUSION_CORE;
}

static void fusionUpdate(df3FusionJob_t *j, const float *z, const float *R, float threshold, unsigned after)
{
    df3CoreStartUpdate(&j->coreJob, z, R, j->h, j->H, threshold);
    j->operation = 1;
    j->afterCore = after;
    j->phase = FUSION_CORE;
}

static void fusionZero(df3FusionJob_t *j, df3Estimator_t *e, df3Observation_e kind, const float *R, unsigned after)
{
    const float zero[3] = {0};
    if (!df3ModelObserve(e->core.x, kind, 0, j->h, j->H)) {
        fusionFail(j, e);
        return;
    }
    fusionUpdate(j, zero, R, 16.27f, after);
}

#ifdef USE_DF3_MULTIRATE
#include "df3_multirate_estimator.inc"
#endif

void df3FusionResume(df3FusionJob_t *j, df3Estimator_t *e)
{
    if (!j || !e || !j->busy) {
        return;
    }
    if (e->failed || !e->initialized) {
        fusionFail(j, e);
        return;
    }
    ++j->slices;
    const df3Event_t *event = j->index < j->count ? &j->batch[j->index] : NULL;
    const float zero[3] = {0};
    float *x = e->core.x;
    switch (j->phase) {
    case FUSION_SELECT:
        if (event) {
            fusionPredict(j, e, event->us, FUSION_EVENT);
        } else if (j->horizonUs > e->fusionUs) {
            fusionPredict(j, e, j->horizonUs, FUSION_PUBLISH);
        } else {
            j->phase = FUSION_PUBLISH;
        }
        return;
    case FUSION_CORE:
        if (!df3CoreResume(&j->coreJob, &e->core, &e->work)) {
            return;
        }
        if (j->coreJob.status != DF3_OK && (j->operation == 0 || j->coreJob.status != DF3_REJECTED)) {
#ifdef USE_DF3_MULTIRATE
            if (!j->operation && j->coreJob.status == DF3_INVALID_ARGUMENT) {
                ++j->historyFaults;
            }
#endif
            fusionFail(j, e);
            return;
        }
        if (!j->operation) {
            ++e->predictions;
            e->fusionUs = j->predictionUs;
        } else if (j->coreJob.status == DF3_OK) {
            ++e->updates;
        } else {
            ++e->rejected;
        }
        j->phase = j->afterCore;
        return;
    case FUSION_EVENT:
        switch (event->kind) {
        case DF3_EVENT_IMU:
#ifdef USE_DF3_MULTIRATE
            e->stationary = !e->dynamicsActive && event->data[0] == 1.f;
            j->stationaryDue = e->stationary &&
#if DF3_FUSION_HZ == 30
                               /* Production: visit the first window start after each stationary-clock
                                * boundary. Consecutive starts differ by one epoch, so this detects a
                                * crossing without additional state. The 50 ms clock stays independent
                                * of 33.334 ms fusion (alternating one/two-window spacing). */
                               (event->us - DF3_EPOCH_US / 2) % DF3_STATIONARY_PERIOD_US < DF3_EPOCH_US;
#else
                               /* Research: 100 Hz epochs divide the stationary period exactly. */
                               (event->us - DF3_EPOCH_US / 2) % DF3_STATIONARY_PERIOD_US == 0;
#endif
            if (j->stationaryDue) {
                ++j->stationaryUpdates;
            }
            if (j->stationaryDue)
#else
            if (!e->dynamicsActive)
#endif
                fusionZero(j, e, DF3_ACCELERATION, zeroAccelerationR, FUSION_IMU_OBSERVE);
            else {
                j->phase = FUSION_IMU_OBSERVE;
            }
            return;
        case DF3_EVENT_ATTITUDE: {
            if (!df3ModelObserve(x, DF3_ATTITUDE, event->data, j->h, j->H)) {
                fusionFail(j, e);
                return;
            }
            float residual[3];
            for (unsigned i = 0; i < 3; ++i) {
                residual[i] = -j->h[i];
            }
            df3RobustAttitudeCovariance(residual, e->core.P, attitudeR, j->R);
            fusionUpdate(j, zero, j->R, 16.27f, FUSION_EVENT_DONE);
            return;
        }
        case DF3_EVENT_RANGE: {
            const float range = event->data[0], strength = event->data[1];
            j->phase = FUSION_EVENT_DONE;
            if (range >= -.001f || range <= -20) {
                return;
            }
            const bool moving = e->dynamicsActive
#ifdef USE_DF3_MULTIRATE
                                || !e->stationary
#endif
                ;
            const df3RangeDecision_t decision =
                df3RangeObserve(&e->terrain, range, event->us, x[2], moving ? x[5] : 0, strength, e->core.P[2 * 18 + 2],
                                moving ? e->core.P[5 * 18 + 5] : 0);
            e->lastRangeDecision = decision;
            e->lastRange = range;
            e->lastStrength = strength;
            e->rangeUs = event->us;
            if (!e->dynamicsActive
#ifdef USE_DF3_MULTIRATE
                && e->stationary
#endif
            ) {
                if (decision.surfaceLocked) {
                    x[2] = range;
                    df3RangeReset(&e->terrain, range, event->us, strength);
                    e->verticalReferenceUs = event->us;
                    e->verticalUpdates = 3;
                }
            } else if (decision.hasPosition) {
                const float z[3] = {x[0], x[1], decision.position};
                const float R[9] = {1e6f, 0, 0, 0, 1e6f, 0, 0, 0, decision.positionVariance};
                if (!df3ModelObserve(x, DF3_POSITION, 0, j->h, j->H)) {
                    fusionFail(j, e);
                    return;
                }
                fusionUpdate(j, z, R, 0, FUSION_RANGE_DONE);
            }
            return;
        }
        case DF3_EVENT_FLOW:
            j->phase = FUSION_EVENT_DONE;
            if (
#ifdef USE_DF3_MULTIRATE
                (!e->dynamicsActive && e->stationary) ||
#else
                !e->dynamicsActive ||
#endif
                event->us < e->imuUs || event->us - e->imuUs > 200000 || event->us < e->attitudeUs ||
                event->us - e->attitudeUs > 250000)
                return;
            if (!df3FlowObserveOffset(x, event->data[5], e->flowSensorOffset, j->h, j->H)) {
                return;
            }
            j->z[0] = event->data[0];
            j->z[1] = event->data[1];
            j->z[2] = 0;
            memset(j->prior, 0, sizeof(j->prior));
            j->priorAxis = 0;
            j->phase = FUSION_PRIOR;
            return;
        case DF3_EVENT_ACTIVATE:
            if (!e->dynamicsActive) {
                e->dynamicsActive = true;
                if (event->us >= e->rangeUs && event->us - e->rangeUs <= 300000) {
                    df3RangeReset(&e->terrain, e->lastRange, e->rangeUs, e->lastStrength);
                }
            }
            j->phase = FUSION_EVENT_DONE;
            return;
        default:
            fusionFail(j, e);
            return;
        }
    case FUSION_IMU_OBSERVE:
#ifdef USE_DF3_MULTIRATE
        multirateWindowBegin(j, e, event);
        return;
    case FUSION_WINDOW_BACK:
        multirateWindowBack(j, e, event);
        return;
    case FUSION_WINDOW_FORWARD:
        multirateWindowForward(j, e, event);
        return;
    case FUSION_WINDOW_FINISH:
        df3WindowObservation(x, j->windowM, j->windowBiasH, j->h, j->H);
#else
        if (!df3ModelObserve(x, DF3_BODY_ACCELEROMETER, 0, j->h, j->H)) {
            fusionFail(j, e);
            return;
        }
#endif
        memset(j->prior, 0, sizeof(j->prior));
        j->priorAxis = 0;
        j->phase = FUSION_PRIOR;
        return;
    case FUSION_PRIOR: {
        const unsigned axis = j->priorAxis;
        for (unsigned i = 0; i < DF3_NE; ++i) {
            if (j->H[axis * DF3_NE + i] == 0) {
                continue;
            }
            for (unsigned k = 0; k < DF3_NE; ++k) {
                j->prior[axis] += j->H[axis * DF3_NE + i] * e->core.P[i * DF3_NE + k] * j->H[axis * DF3_NE + k];
            }
        }
        if (++j->priorAxis == (event->kind == DF3_EVENT_IMU ? 3u : 2u)) {
            j->phase = event->kind == DF3_EVENT_IMU ? FUSION_IMU_UPDATE : FUSION_FLOW_UPDATE;
        }
        return;
    }
    case FUSION_IMU_UPDATE:
        if (!event) {
            fusionFail(j, e);
            return;
        }
        df3RobustAccelerometerCovariance(event->data + 3, j->h, j->prior, accelerationR, j->R);
#ifdef USE_DF3_MULTIRATE
        // Retain lateral roughness weighting. Z uses its base covariance and
        // innovation-based outlier weighting: high-rate force roughness is not
        // the variance of the window mean, and inflating Z from it delays the
        // vertical velocity response. The measured force mean is unchanged.
        j->R[0] += 1000.f * event->data[1];
        j->R[4] += 1000.f * event->data[1];
#endif
        fusionUpdate(j, event->data + 3, j->R, 16.27f, FUSION_IMU_ZERO_VEL);
        return;
    case FUSION_IMU_ZERO_VEL:
#ifdef USE_DF3_MULTIRATE
        if (j->stationaryDue)
#else
        if (!e->dynamicsActive)
#endif
            fusionZero(j, e, DF3_VELOCITY, zeroVelocityR, FUSION_EVENT_DONE);
        else {
            j->phase = FUSION_EVENT_DONE;
        }
        return;
    case FUSION_FLOW_UPDATE: {
        const df3FlowNoise_t noise = {
            .pRadps = event->data[2] - x[DF3_BG], .qRadps = event->data[3] - x[DF3_BG + 1],
            .quality = event->data[4], .height = event->data[5] * e->flowHeightPerGain,
            .rangeVariance = df3RangeMeasurementVariance(&e->terrain, event->data[6])};
        df3RobustFlowCovariance(j->z, j->h, j->prior, &noise, j->R);
    }
#ifdef USE_DF3_MULTIRATE
        df3FlowQuantizedStart(&j->quantized, e->flowQuantumPerGain * event->data[5]);
        j->phase = FUSION_FLOW_QUANTIZED;
        return;
    case FUSION_FLOW_QUANTIZED: {
        const int done = df3FlowQuantizedResume(&j->quantized, j->prior, j->z, j->h, j->H, j->R);
        if (!done) {
            return;
        }
        if (done < 0) {
#else
        if (!df3FlowQuantizedObservation(e->flowQuantumPerGain * event->data[5], j->prior, j->z, j->h, j->H, j->R)) {
#endif
            j->phase = FUSION_EVENT_DONE;
            return;
        }
        fusionUpdate(j, j->z, j->R, 16.27f, FUSION_EVENT_DONE);
        return;
#ifdef USE_DF3_MULTIRATE
    }
#endif
    case FUSION_RANGE_DONE:
        if (e->lastRangeDecision.surfaceLocked) {
            e->verticalReferenceUs = event->us;
            if (e->verticalUpdates < 3) {
                ++e->verticalUpdates;
            }
        }
        j->phase = FUSION_EVENT_DONE;
        return;
    case FUSION_EVENT_DONE:
        if (event->kind == DF3_EVENT_IMU) {
#ifndef USE_DF3_MULTIRATE
            memcpy(e->gyro, event->data, sizeof(e->gyro));
#endif
            e->imuUs = event->us;
        } else if (event->kind == DF3_EVENT_ATTITUDE) {
            e->attitudeUs = event->us;
        }
        if (!df3CoreHealthy(&e->core)) {
            fusionFail(j, e);
            return;
        }
        ++j->index;
        j->phase = FUSION_SELECT;
        return;
    case FUSION_PUBLISH:
#ifdef USE_DF3_MULTIRATE
        /* The expensive lag-to-present replay belongs to the budgeted worker.
     * Its target is frozen: arrivals while paused must not move the goal. */
        fusionSnapshot(&j->projected, e);
        j->projectionTargetUs = j->lastNowUs;
        if (j->projectionTargetUs < j->projected.timeUs) {
            fusionFail(j, e);
            return;
        }
        j->phase = FUSION_PROJECT;
        return;
    case FUSION_PROJECT: {
        const uint64_t cursor = j->projected.timeUs;
        if (cursor < j->projectionTargetUs) {
            float gyro[3];
            uint64_t next;
            if (!df3GyroHistoryIntervalHint(j->history, cursor, j->projectionTargetUs, true, gyro, &next, NULL,
                                            &j->historyHint) ||
                !df3ProjectNominal(j->projected.x, gyro, (float)(next - cursor) * 1e-6f)) {
                ++j->historyFaults;
                fusionFail(j, e);
                return;
            }
            j->projected.timeUs = next;
            if (next < j->projectionTargetUs) {
                return;
            }
        }
        /* Publish state and fused timestamps together, never the partial replay. */
        j->committed = j->projected;
        fusionPublishMetadata(j, e);
        j->busy = false;
        ++j->transactions;
        ++j->epochs;
        return;
    }
#else
        fusionPublish(j, e);
        j->busy = false;
        ++j->transactions;
        return;
#endif
    default:
        fusionFail(j, e);
        return;
    }
}

#ifdef USE_DF3_MULTIRATE
static bool fusionObserveOutput(df3FusionJob_t *j, df3Estimate_t *out)
{
    if (!j->outputTimeUs) {
        memcpy(j->outputMotion, out->x, sizeof(j->outputMotion));
        j->outputTimeUs = out->timeUs;
        return true;
    }
    if (out->timeUs < j->outputTimeUs) {
        return false;
    }
    const float dt = (float)(out->timeUs - j->outputTimeUs) * 1e-6f;
    /* Preserve predicted motion; relax only its disagreement with the latest
     * corrected estimate. A consistent constant-acceleration trajectory has
     * no filter lag. Fusion-induced P/V steps use a 15 ms time constant;
     * longer correction times reduce damping in the closed-loop noise tests.
     * Elapsed time, not poll count, sets the gain. One exp and three axes;
     * no replay, covariance work, heap or new scheduler phase. */
    const float gain = 1.f - expf(-dt / .015f);
    float next[9];
    for (unsigned i = 0; i < 3; ++i) {
        const float position = j->outputMotion[i] + j->outputMotion[DF3_V + i] * dt +
                               .5f * j->outputMotion[DF3_A + i] * dt * dt;
        const float velocity = j->outputMotion[DF3_V + i] + j->outputMotion[DF3_A + i] * dt;
        next[i] = position + gain * (out->x[i] - position);
        next[DF3_V + i] = velocity + gain * (out->x[DF3_V + i] - velocity);
        next[DF3_A + i] = out->x[DF3_A + i];
        if (!isfinite(next[i]) || !isfinite(next[DF3_V + i]) || !isfinite(next[DF3_A + i])) {
            return false;
        }
    }
    memcpy(j->outputMotion, next, sizeof(next));
    j->outputTimeUs = out->timeUs;
    memcpy(out->x, next, 6 * sizeof(float));
    return true;
}
#endif

bool df3FusionOutput(df3FusionJob_t *j, const df3Estimator_t *e, uint64_t now, df3Estimate_t *out)
{
    DF3_PROFILE_SCOPE(DF3_PROF_OUTPUT);
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->timeUs = now;
    if (!j || !e || !j->published || e->failed || !e->initialized || now < e->lastTickUs || now < e->bootstrapUs ||
        now < j->committed.covarianceTimeUs) {
        return false;
    }
    *out = j->committed;
    out->valid = out->verticalReferenceValid = false;
#ifdef USE_DF3_MULTIRATE
    /* Current-output cache is separate from the worker's private replay. Only
   * the short interval since its last successful output remains in foreground.
   * Never advance a cache partially or label an old state with a new time. */
    df3Estimate_t candidate = j->committed;
    uint64_t cursor = candidate.timeUs;
    if (cursor > now) {
        return false;
    }
    unsigned steps = 0;
    while (cursor < now && steps++ < 8) {
        float gyro[3];
        uint64_t next;
        if (!df3GyroHistoryIntervalHint(j->history, cursor, now, true, gyro, &next, NULL, &j->outputHistoryHint) ||
            !df3ProjectNominal(candidate.x, gyro, (float)(next - cursor) * 1e-6f)) {
            return false;
        }
        cursor = next;
    }
    if (cursor != now) {
        return false;
    }
    candidate.timeUs = now;
    j->committed = candidate;
    *out = candidate;
#else
    /* Research: project from queued IMU events rather than dedicated gyro history. */
    out->timeUs = now;
    uint64_t projected = j->committed.covarianceTimeUs;
    float gyro[3];
    memcpy(gyro, j->committedGyro, sizeof(gyro));
    /* Retain the entire active batch for projection, including measurements
   * already consumed by the worker since the last publication. */
    for (unsigned list = 0; list < 2; ++list) {
        const df3Event_t *events = list == 0 ? j->batch : e->events;
        const unsigned count = list == 0 ? (j->busy ? j->count : 0) : e->count;
        for (unsigned i = 0; i < count; ++i) {
            const df3Event_t *event = &events[i];
            if (event->us > now) {
                break;
            }
            if (event->kind != DF3_EVENT_IMU || event->us < projected) {
                continue;
            }
            if (!df3ProjectNominal(out->x, gyro, (float)(event->us - projected) * 1e-6f)) {
                return false;
            }
            projected = event->us;
            memcpy(gyro, event->data, sizeof(gyro));
        }
    }
    if (now < projected || now - projected > 200000 ||
        !df3ProjectNominal(out->x, gyro, (float)(now - projected) * 1e-6f)) {
        return false;
    }
#endif
    out->valid = j->committed.valid && now - e->bootstrapUs >= 150000 && now >= j->committedImuUs &&
                 now - j->committedImuUs <= 200000 && now >= j->committedAttitudeUs &&
                 now - j->committedAttitudeUs <= 250000;
#ifdef USE_DF3_MULTIRATE
    if (out->valid && !fusionObserveOutput(j, out)) {
        out->valid = false;
    }
#endif
    out->verticalReferenceValid = out->valid && now >= j->committedVerticalUs && now - j->committedVerticalUs <= 500000;
    return out->valid;
}
