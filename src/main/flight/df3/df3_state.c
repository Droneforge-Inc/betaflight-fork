#include "df3_state.h"
#include <math.h>
#include <string.h>
static bool quantize(uint8_t *p, float value, float scale)
{
    float v = value * scale;
    if (!isfinite(v) || v < -32768 || v > 32767) {
        return false;
    }
    df3WriteU16Be(p, (uint16_t)(int16_t)lrintf(v));
    return true;
}
void df3StateEncode(const df3Estimate_t *s, uint16_t epoch, uint16_t sequence, bool armed, bool assist,
                    uint8_t p[DF3_STATE_BYTES])
{
    memset(p, 0, DF3_STATE_BYTES);
    p[DF3_WIRE_VERSION] = DF3_PROTOCOL_VERSION;
    p[DF3_WIRE_FLAGS] = (armed ? DF3_STATE_ARMED : 0) | (assist ? DF3_STATE_ASSIST : 0);
    df3WriteU16Be(p + DF3_WIRE_EPOCH, epoch);
    df3WriteU16Be(p + DF3_WIRE_SEQUENCE, sequence);
    const uint32_t ms = (uint32_t)(s->timeUs / 1000);
    df3WriteU16Be(p + DF3_WIRE_SOURCE_MS, (uint16_t)(ms >> 16));
    df3WriteU16Be(p + DF3_WIRE_SOURCE_MS + 2, (uint16_t)ms);
    bool valid = s->valid && epoch;
    for (unsigned i = 0; i < 9; ++i) {
        valid = quantize(p + DF3_WIRE_POSITION + 2 * i, s->x[i], i < 3 ? 100 : 1000) && valid;
    }
    const float *q = s->x + DF3_Q;
    const float norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    const float sinPitch = 2 * (q[0] * q[2] - q[3] * q[1]);
    const float angles[] = {atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])),
                            asinf(fmaxf(-1, fminf(1, sinPitch))),
                            atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]))};
    valid = valid && isfinite(norm) && fabsf(norm - 1) < .01f;
    for (unsigned i = 0; i < 3; ++i) {
        valid = quantize(p + DF3_WIRE_ANGLES + 2 * i, angles[i], 1000) && valid;
    }
    if (valid) {
        p[DF3_WIRE_FLAGS] |= DF3_STATE_VALID | (s->verticalReferenceValid ? DF3_STATE_VERTICAL_VALID : 0);
    } else {
        memset(p + DF3_WIRE_POSITION, 0,
               DF3_STATE_BYTES - DF3_WIRE_HEADER_BYTES); // Never send saturated values as valid state.
    }
}
