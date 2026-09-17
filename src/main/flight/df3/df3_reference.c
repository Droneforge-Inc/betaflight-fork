#include "df3_reference.h"
#include <math.h>
#include <string.h>

void df3ReferenceReset(df3ReferenceReceiver_t *r)
{
    memset(r, 0, sizeof(*r));
}

bool df3ReferenceDecode(const uint8_t p[DF3_REFERENCE_BYTES], df3Reference_t *out)
{
    if (!p || !out || p[DF3_WIRE_VERSION] != DF3_PROTOCOL_VERSION || p[DF3_WIRE_FLAGS] > DF3_REFERENCE_INACTIVE) {
        return false;
    }
    df3Reference_t value = {.epoch = df3ReadU16Be(p + DF3_WIRE_EPOCH),
                            .sequence = df3ReadU16Be(p + DF3_WIRE_SEQUENCE),
                            .sourceMs = df3ReadU32Be(p + DF3_WIRE_SOURCE_MS),
                            .yaw = df3ReadI16Be(p + DF3_WIRE_REFERENCE_YAW) * .001f,
                            .leaseMs = df3ReadU16Be(p + DF3_WIRE_REFERENCE_LEASE),
                            .active = p[DF3_WIRE_FLAGS] == 0};
    if (!value.epoch || value.leaseMs < DF3_REFERENCE_MIN_LEASE_MS || value.leaseMs > DF3_REFERENCE_MAX_LEASE_MS ||
        fabsf(value.yaw) > 3.142f) {
        return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        value.position[i] = df3ReadI16Be(p + DF3_WIRE_POSITION + 2 * i) * .01f;
        value.velocity[i] = df3ReadI16Be(p + DF3_WIRE_VELOCITY + 2 * i) * .001f;
        value.acceleration[i] = df3ReadI16Be(p + DF3_WIRE_ACCELERATION + 2 * i) * .001f;
    }
    *out = value;
    return true;
}

bool df3ReferenceAccept(df3ReferenceReceiver_t *r, const uint8_t p[DF3_REFERENCE_BYTES], uint64_t receivedUs,
                        bool armed)
{
    if (!r) {
        return false;
    }
    df3Reference_t value;
    if (!df3ReferenceDecode(p, &value)) {
        ++r->rejected;
        return false;
    }
    uint64_t mappedUs = receivedUs;
    if (!r->initialized || value.epoch != r->reference.epoch) {
        // Establish a new trajectory coordinate/time epoch only while disarmed.
        if (armed) {
            ++r->rejected;
            return false;
        }
    } else {
        const uint16_t sequenceDelta = (uint16_t)(value.sequence - r->reference.sequence);
        const uint32_t sourceDelta = value.sourceMs - r->reference.sourceMs;
        if (!sequenceDelta || sequenceDelta >= 32768 || !sourceDelta || sourceDelta >= 0x80000000U ||
            receivedUs < r->receivedUs ||
            (uint64_t)sourceDelta * 1000 > receivedUs - r->receivedUs + DF3_REFERENCE_MAX_SOURCE_LEAD_US) {
            ++r->rejected;
            return false;
        }
        mappedUs = r->sourceLocalUs + (uint64_t)sourceDelta * 1000;
        // Minimum observed clock offset: faster delivery improves the mapping.
        // This detects queue growth, but does not identify absolute one-way
        // latency of the fastest packet. Receipt TTL is enforced separately.
        if (mappedUs > receivedUs) {
            mappedUs = receivedUs;
        }
        if (receivedUs - mappedUs > (uint64_t)value.leaseMs * 1000) {
            ++r->rejected;
            return false;
        }
    }
    r->reference = value;
    r->receivedUs = receivedUs;
    r->sourceLocalUs = mappedUs;
    r->initialized = true;
    ++r->accepted;
    return true;
}

bool df3ReferenceSample(const df3ReferenceReceiver_t *r, uint64_t nowUs, df3Reference_t *out)
{
    if (!r || !out || !r->initialized || !r->reference.active || nowUs < r->receivedUs || nowUs < r->sourceLocalUs ||
        nowUs - r->receivedUs > (uint64_t)r->reference.leaseMs * 1000 ||
        nowUs - r->sourceLocalUs > (uint64_t)r->reference.leaseMs * 1000) {
        return false;
    }
    *out = r->reference;
    const float dt = (float)(nowUs - r->sourceLocalUs) * 1e-6f;
    for (unsigned i = 0; i < 3; ++i) {
        out->position[i] += out->velocity[i] * dt + .5f * out->acceleration[i] * dt * dt;
        out->velocity[i] += out->acceleration[i] * dt;
    }
    return true;
}
