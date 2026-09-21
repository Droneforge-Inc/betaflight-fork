/* Production: sensor-window frontend, included by df3_flow.c.
 * Reduce held accelerometer samples into duration-weighted observations at
 * window midpoints, admit attitude samples once per epoch, and track stationary
 * eligibility. The same implementation serves 30 Hz production and retained
 * 100 Hz research builds. */
#include "df3_multirate.h"

void df3ImuReducerReset(df3ImuReducer_t *r)
{
    memset(r, 0, sizeof(*r));
    r->attitudeBin = UINT64_MAX;
}

bool df3AttitudeAdmit(df3ImuReducer_t *r, uint64_t us)
{
    const uint64_t bin = us / DF3_EPOCH_US;
    if (r->attitudeBin != UINT64_MAX && bin <= r->attitudeBin) {
        return false;
    }
    r->attitudeBin = bin;
    return true;
}

int df3ImuReducerPush(df3ImuReducer_t *r, uint64_t us, const float gyro[3], const float force[3], bool inactive,
                      bool externalStill, float range, df3Event_t *out)
{
    if (!r || !gyro || !force || !out) {
        return -1;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(gyro[i]) || !isfinite(force[i]) || fabsf(force[i]) > 156.91f) {
            ++r->invalid;
            r->haveSample = false;
            r->stationary = false;
            r->stationarySinceUs = 0;
            return -1;
        }
    }
    if (r->haveSample && (us <= r->lastUs || us - r->lastUs > DF3_RAW_IMU_MAX_GAP_US)) {
        ++r->gaps;
        r->haveSample = false;
        r->stationary = false;
        r->stationarySinceUs = 0;
        return -1;
    }
    /* Stationary eligibility requires inactive dynamics, 250 ms of quiet
     * inertial data, the caller's externalStill check, and stable range.
     * Any violated check exits immediately, including lifting while disarmed. */
    float g2 = 0, w2 = 0, change2 = 0;
    for (unsigned i = 0; i < 3; ++i) {
        g2 += force[i] * force[i];
        w2 += gyro[i] * gyro[i];
        const float d = force[i] - r->held[i];
        change2 += d * d;
    }
    bool quiet = inactive && externalStill && isfinite(range) && w2 < .0225f && fabsf(sqrtf(g2) - 9.80665f) < .5f &&
                 (!r->haveSample || change2 < .25f);
    if (quiet && r->stationarySinceUs && fabsf(range - r->rangeAnchor) > .025f) {
        quiet = false;
    }
    if (!quiet) {
        r->stationarySinceUs = 0;
        r->stationary = false;
    } else if (!r->stationarySinceUs) {
        r->stationarySinceUs = us;
        r->rangeAnchor = range;
    } else {
        r->stationary = us - r->stationarySinceUs >= 250000;
    }

    if (!r->haveSample) {
        r->startUs = ((us + DF3_EPOCH_US - 1) / DF3_EPOCH_US) * DF3_EPOCH_US;
        if (r->startUs != us) {
            ++r->partial;
        }
        memset(r->sum, 0, sizeof(r->sum));
        memset(r->forceDerivative, 0, sizeof(r->forceDerivative));
        memset(r->forceRoughness, 0, sizeof(r->forceRoughness));
        r->derivativeDt = 0;
        r->haveSample = true;
        r->lastUs = us;
        memcpy(r->held, force, sizeof(r->held));
        return 0;
    }
    /* Second time difference rejects constant force and constant jerk.
     * Normalize to one millisecond and smooth the energy by elapsed time.
     * The tuned input is the FC accelerometer after its 25 Hz low-pass.
     * This is a noise proxy, not an acceleration/bias observation. */
    const float sensorDt = (float)(us - r->lastUs) * 1e-6f;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const float derivative = (force[axis] - r->held[axis]) / sensorDt;
        if (r->derivativeDt > 0) {
            const float roughness = (derivative - r->forceDerivative[axis]) *
                                    (.000002f / (sensorDt + r->derivativeDt));
            const float energy = fminf(100.f, roughness * roughness);
            r->forceRoughness[axis] += sensorDt / (.05f + sensorDt) *
                                      (energy - r->forceRoughness[axis]);
        }
        r->forceDerivative[axis] = derivative;
    }
    r->derivativeDt = sensorDt;
    const uint64_t boundary = r->startUs + DF3_EPOCH_US;
    const uint64_t begin = r->lastUs > r->startUs ? r->lastUs : r->startUs;
    const uint64_t end = us < boundary ? us : boundary;
    if (end > begin) {
        for (unsigned i = 0; i < 3; ++i) {
            r->sum[i] += r->held[i] * (float)(end - begin);
        }
    }
    int result = 0;
    if (us >= boundary) {
        *out = (df3Event_t){.us = r->startUs + DF3_EPOCH_US / 2, .kind = DF3_EVENT_IMU};
        out->data[0] = r->stationary ? 1.f : 0.f;
        out->data[1] = fmaxf(r->forceRoughness[0], r->forceRoughness[1]);
        out->data[2] = r->forceRoughness[2];
        for (unsigned i = 0; i < 3; ++i) {
            out->data[i + 3] = r->sum[i] / DF3_EPOCH_US;
            r->sum[i] = r->held[i] * (float)(us - boundary);
        }
        r->startUs = boundary;
        ++r->windows;
        result = 1;
    }
    r->lastUs = us;
    memcpy(r->held, force, sizeof(r->held));
    return result;
}
