#include "df3_flow.h"
#include "df3_profile.h"
#include <math.h>
#include <string.h>

bool df3FlowQuantizedObservation(float step, const float prior[2], float z[3], const float h[3], float H[3 * DF3_NE],
                                 float R[9])
{
    if (!isfinite(step) || step < 0) {
        return false;
    }
    if (step == 0) {
        return true;
    }
    if (!prior || !z || !h || !H || !R) {
        return false;
    }
    float observation[2], variance[2];
    bool unobserved[2] = {false, false};
    for (unsigned axis = 0; axis < 2; ++axis) {
        const float p = fmaxf(0, prior[axis]), r = R[4 * axis], s = p + r;
        if (!isfinite(prior[axis]) || !isfinite(z[axis]) || !isfinite(h[axis]) || !isfinite(s) || r <= 0 ||
            !isfinite(r)) {
            return false;
        }
        observation[axis] = z[axis];
        variance[axis] = r;
        const float widthSquared = step * step / s;
        if (!isfinite(widthSquared)) {
            return false;
        }
        const float residual = z[axis] - h[axis], centerSquared = residual * residual / s;
        /* Fourth-order narrow-bin expansion of the conditional moments.
         * First order is R += step^2/12; the next term also adjusts innovation.
         * The common 1- and 10-count paths avoid sqrt/exp entirely. */
        if (widthSquared <= .125f && centerSquared <= 9) {
            variance[axis] += step * step * (1.f / 12.f) * (1 + widthSquared * (1 - centerSquared) * .05f);
            observation[axis] -= residual * centerSquared * widthSquared * widthSquared * (1.f / 360.f);
            continue;
        }
        const float sd = sqrtf(s), center = (z[axis] - h[axis]) / sd;
        const float half = .5f * step / sd;
        const float lo = fmaxf(-8, center - half), hi = fminf(8, center + half);
        /* An extreme tail stays on the existing robust/gated path. Do not
         * manufacture a posterior from an underflowing likelihood. */
        if (lo >= hi) {
            continue;
        }
        if (lo == -8 && hi == 8) {
            unobserved[axis] = true;
            observation[axis] = h[axis];
            continue;
        }
        const float peak = fmaxf(lo, fminf(0, hi));
        static const float nodes[5] = {-.9061798459f, -.5384693101f, 0, .5384693101f, .9061798459f};
        static const float weights[5] = {.2369268851f, .4786286705f, .5688888889f, .4786286705f, .2369268851f};
        /* Five-point Gauss-Legendre on segments no wider than one sigma.
         * Bounded by 16 segments. Centered moments avoid tail cancellation. */
        const unsigned segments = (unsigned)ceilf(hi - lo);
        const float halfSegment = .5f * (hi - lo) / (float)segments;
        float mass = 0, first = 0, second = 0;
        for (unsigned k = 0; k < segments; ++k) {
            const float mid = lo + (2.f * (float)k + 1) * halfSegment;
            for (unsigned j = 0; j < 5; ++j) {
                const float y = mid + halfSegment * nodes[j], t = y - center;
                const float w = weights[j] * expf(-.5f * (y * y - peak * peak));
                mass += w;
                first += w * t;
                second += w * t * t;
            }
        }
        if (!isfinite(mass) || mass <= 0) {
            return false;
        }
        const float mean = first / mass;
        const float conditionalVariance = fmaxf(0, second / mass - mean * mean);
        const float information = 1 - fminf(1, conditionalVariance);
        if (information <= .00001f) {
            unobserved[axis] = true;
            observation[axis] = h[axis];
        } else {
            observation[axis] = h[axis] + sd * (center + mean) / information;
            variance[axis] = fmaxf(r, s / information - p);
        }
        if (!isfinite(observation[axis]) || !isfinite(variance[axis])) {
            return false;
        }
    }
    for (unsigned axis = 0; axis < 2; ++axis) {
        z[axis] = observation[axis];
        R[4 * axis] = variance[axis];
        if (unobserved[axis]) {
            memset(H + axis * DF3_NE, 0, DF3_NE * sizeof(float));
        }
    }
    return true;
}

bool df3FlowObserve(const float x[DF3_NX], float gain, float h[3], float H[3 * DF3_NE])
{
    const float offset[3] = {0};
    return df3FlowObserveOffset(x, gain, offset, h, H);
}

bool df3FlowObserveOffset(const float x[DF3_NX], float gain, const float offset[3], float h[3], float H[3 * DF3_NE])
{
    if (!x || !h || !H || !offset || !isfinite(gain) || gain <= 0 || !isfinite(offset[0]) || !isfinite(offset[1]) ||
        !isfinite(offset[2])) {
        return false;
    }
    float R[9], v[3];
    df3QuaternionMatrix(x + DF3_Q, R);
    for (unsigned i = 0; i < 3; ++i) {
        v[i] = R[i] * x[DF3_V] + R[3 + i] * x[DF3_V + 1] + R[6 + i] * x[DF3_V + 2];
    }
    memset(H, 0, 3 * DF3_NE * sizeof(float));
    const float pitchRollGain = gain + offset[2];
    h[0] = v[0] - pitchRollGain * x[DF3_BG + 1] + offset[1] * x[DF3_BG + 2];
    h[1] = v[1] + pitchRollGain * x[DF3_BG] - offset[0] * x[DF3_BG + 2];
    h[2] = 0;
    for (unsigned i = 0; i < 2; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            H[i * DF3_NE + DF3_EV + j] = R[3 * j + i];
        }
    }
    // Right attitude perturbation: d(R^T v)/d(theta) = [R^T v]_cross.
    H[DF3_ET + 1] = -v[2];
    H[DF3_ET + 2] = v[1];
    H[DF3_NE + DF3_ET] = v[2];
    H[DF3_NE + DF3_ET + 2] = -v[0];
    H[DF3_EBG + 1] = -pitchRollGain;
    H[DF3_EBG + 2] = offset[1];
    H[DF3_NE + DF3_EBG] = pitchRollGain;
    H[DF3_NE + DF3_EBG + 2] = -offset[0];
    return isfinite(h[0]) && isfinite(h[1]);
}

void df3GyroHistoryReset(df3GyroHistory_t *h)
{
    memset(h, 0, sizeof(*h));
}

static const df3GyroSample_t *sample(const df3GyroHistory_t *h, unsigned i)
{
    return &h->samples[(h->first + i) % DF3_GYRO_HISTORY_CAPACITY];
}

bool df3GyroHistoryPush(df3GyroHistory_t *h, uint64_t us, const float gyro[3])
{
    if (!h || !gyro || !isfinite(gyro[0]) || !isfinite(gyro[1]) || !isfinite(gyro[2])) {
        return false;
    }
    if (h->count && us <= sample(h, h->count - 1)->us) {
        return false;
    }
    if (h->count == DF3_GYRO_HISTORY_CAPACITY) {
        h->first = (h->first + 1) % DF3_GYRO_HISTORY_CAPACITY;
        --h->count;
    }
    df3GyroSample_t *s = &h->samples[(h->first + h->count) % DF3_GYRO_HISTORY_CAPACITY];
    s->us = us;
    memcpy(s->gyro, gyro, sizeof(s->gyro));
    ++h->count;
    return true;
}

bool df3GyroHistoryAverage(const df3GyroHistory_t *h, uint64_t start, uint64_t end, float average[3])
{
    if (!h || !average || h->count < 2 || end <= start || end - start > 200000 || start < sample(h, 0)->us ||
        end > sample(h, h->count - 1)->us) {
        return false;
    }
    float sum[3] = {0};
    uint64_t covered = 0;
    for (unsigned i = 0; i + 1 < h->count; ++i) {
        const df3GyroSample_t *left = sample(h, i), *right = sample(h, i + 1);
        if (left->us >= end) {
            break;
        }
        if (right->us <= start) {
            continue;
        }
        /* A missing gyro interval is not an indefinitely valid held sample. */
        if (right->us - left->us > 10000) {
            return false;
        }
        const uint64_t lo = left->us > start ? left->us : start;
        const uint64_t hi = right->us < end ? right->us : end;
        const uint64_t duration = hi - lo;
        covered += duration;
        for (unsigned j = 0; j < 3; ++j) {
            sum[j] += left->gyro[j] * (float)duration;
        }
    }
    if (covered != end - start) {
        return false;
    }
    for (unsigned j = 0; j < 3; ++j) {
        average[j] = sum[j] / (float)covered;
    }
    return true;
}

bool df3FlowCompensate(const df3GyroHistory_t *h, uint64_t start, uint64_t end, const float flow[2], float clearance,
                       float scale, const float offset[3], df3FlowResult_t *out)
{
    DF3_PROFILE_SCOPE(DF3_PROF_FLOW_COMP);
    if (!out || !flow || !offset || !isfinite(flow[0]) || !isfinite(flow[1]) || !isfinite(clearance) ||
        clearance <= 0 || !isfinite(scale) || scale < 0 || !isfinite(offset[0]) || !isfinite(offset[1]) ||
        !isfinite(offset[2])) {
        return false;
    }
    df3FlowResult_t result = {.startUs = start, .endUs = end, .midpointUs = start + (end - start) / 2};
    if (!df3GyroHistoryAverage(h, start, end, result.averageGyro)) {
        return false;
    }
    const float *w = result.averageGyro;
    result.rawBodyVelocity[0] = flow[0] * clearance;
    result.rawBodyVelocity[1] = flow[1] * clearance;
    /* Existing SDK correction: forward += height * BF_pitch_rate * scale;
     * right += height * BF_roll_rate * scale. FRD q = -BF_pitch_rate. */
    result.rotationCorrection[0] = -scale * clearance * w[1];
    result.rotationCorrection[1] = scale * clearance * w[0];
    /* Convert sensor-origin translation to COM: v_COM = v_sensor - w x r. */
    result.leverCorrection[0] = -(w[1] * offset[2] - w[2] * offset[1]);
    result.leverCorrection[1] = -(w[2] * offset[0] - w[0] * offset[2]);
    for (unsigned i = 0; i < 2; ++i) {
        result.correctedBodyVelocity[i] =
            result.rawBodyVelocity[i] + result.rotationCorrection[i] + result.leverCorrection[i];
    }
    *out = result;
    return true;
}

bool df3GyroHistoryIntervalHint(const df3GyroHistory_t *h, uint64_t start, uint64_t end, bool tail, float gyro[3],
                                uint64_t *next, uint64_t *sampleUs, uint16_t *physicalHint)
{
    if (!h || !gyro || !next || !h->count || end <= start || start < sample(h, 0)->us) {
        return false;
    }
    const uint64_t newest = sample(h, h->count - 1)->us;
    if (end > newest && (!tail || end - newest > 10000)) {
        return false;
    }
    const df3GyroSample_t *left;
    uint64_t right = 0;
    unsigned physical;
    bool hasRight;
    /* A physical index is only a guess: another producer can wrap the ring
     * between resumptions, or another replay can move backwards in time.
     * Validate membership plus both timestamp bounds before bypassing search.
     * The history itself remains const and carries no shared mutable cache. */
    if (physicalHint && *physicalHint < DF3_GYRO_HISTORY_CAPACITY) {
        physical = *physicalHint;
        const unsigned candidate = (physical + DF3_GYRO_HISTORY_CAPACITY - h->first) % DF3_GYRO_HISTORY_CAPACITY;
        if (candidate < h->count) {
            left = &h->samples[physical];
            hasRight = candidate + 1 < h->count;
            if (hasRight) {
                right = h->samples[(physical + 1) % DF3_GYRO_HISTORY_CAPACITY].us;
            }
            if (left->us <= start && (!hasRight || start < right)) {
                goto interval;
            }
        }
    }
    {
        unsigned lo = 0, hi = h->count;
        while (lo + 1 < hi) {
            const unsigned mid = lo + (hi - lo) / 2;
            if (sample(h, mid)->us <= start) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        physical = (h->first + lo) % DF3_GYRO_HISTORY_CAPACITY;
        left = &h->samples[physical];
        hasRight = lo + 1 < h->count;
        if (hasRight) {
            right = h->samples[(physical + 1) % DF3_GYRO_HISTORY_CAPACITY].us;
        }
    }
interval:;
    uint64_t stop = end;
    if (hasRight) {
        if (right - left->us > 10000) {
            return false;
        }
        if (right < stop) {
            stop = right;
        }
    } else if (!tail || end - left->us > 10000) {
        return false;
    }
    if (stop <= start) {
        return false;
    }
    memcpy(gyro, left->gyro, sizeof(left->gyro));
    *next = stop;
    if (sampleUs) {
        *sampleUs = left->us;
    }
    if (physicalHint) {
        /* Most replay calls continue at stop, so preselect its interval. A
         * different next start simply fails the bracket check and searches. */
        *physicalHint = (uint16_t)((physical + (hasRight && stop == right)) % DF3_GYRO_HISTORY_CAPACITY);
    }
    return true;
}

bool df3GyroHistoryInterval(const df3GyroHistory_t *h, uint64_t start, uint64_t end, bool tail, float gyro[3],
                            uint64_t *next, uint64_t *sampleUs)
{
    return df3GyroHistoryIntervalHint(h, start, end, tail, gyro, next, sampleUs, NULL);
}

#ifdef USE_DF3_MULTIRATE
#include "df3_multirate_frontend.inc"
#include "df3_multirate_flow.inc"
#endif
