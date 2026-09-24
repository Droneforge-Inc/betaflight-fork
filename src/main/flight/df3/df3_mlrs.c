#include "df3_mlrs.h"
#include "common/df_custom.h"
#include <string.h>

bool df3MlrsCapabilities(size_t requestLength, uint8_t *reply)
{
    if (requestLength || !reply) return false;
    reply[0] = 1;
    reply[1] = DF3_PROTOCOL_VERSION;
    reply[2] = 1;
    reply[3] = 0;
    df3WriteU32Le(reply + 4, FIRMWARE_VERSION_DF);
    df3WriteU32Le(reply + 8, HARDWARE_VERSION_DF);
    return true;
}

bool df3MlrsReferenceDecode(const uint8_t *wire, uint8_t *legacy)
{
    if (!wire || !legacy || wire[0] != 1 || (wire[1] & ~(DF3_MLRS_ACTIVE | DF3_MLRS_AUTONOMY))) {
        return false;
    }
    if (!(wire[1] & DF3_MLRS_ACTIVE)) {
        for (unsigned i = 10; i < 30; ++i) {
            if (wire[i]) return false;
        }
    }
    uint8_t decoded[DF3_REFERENCE_BYTES];
    decoded[0] = DF3_PROTOCOL_VERSION;
    decoded[1] = (wire[1] & DF3_MLRS_ACTIVE) ? 0 : DF3_REFERENCE_INACTIVE;
    for (unsigned i = 2; i < DF3_REFERENCE_BYTES; i += 2) {
        df3WriteU16Be(decoded + i, df3ReadU16Le(wire + i));
    }
    // Source time is one uint32, unlike the surrounding uint16 fields.
    df3WriteU16Be(decoded + 6, df3ReadU16Le(wire + 8));
    df3WriteU16Be(decoded + 8, df3ReadU16Le(wire + 6));
    df3Reference_t reference;
    if (!df3ReferenceDecode(decoded, &reference)) return false;
    memcpy(legacy, decoded, sizeof(decoded));
    return true;
}

bool df3MlrsReferenceAccept(df3ReferenceReceiver_t *r, uint8_t *previous, const uint8_t *wire,
                            uint64_t receivedUs, bool armed)
{
    uint8_t legacy[DF3_REFERENCE_BYTES];
    if (!r || !previous || !df3MlrsReferenceDecode(wire, legacy)) {
        if (r) ++r->rejected;
        return false;
    }
    if (r->initialized && !memcmp(previous, wire, DF3_MLRS_REFERENCE_BYTES)) {
        return true;
    }
    if (!df3ReferenceAccept(r, legacy, receivedUs, armed)) return false;
    memcpy(previous, wire, DF3_MLRS_REFERENCE_BYTES);
    return true;
}

uint8_t df3MlrsAge10ms(uint64_t ageUs, bool valid)
{
    if (!valid || ageUs > 2540000) return UINT8_MAX;
    return (uint8_t)((ageUs + 9999) / 10000);
}

bool df3MlrsReferenceFresh(const df3ReferenceReceiver_t *r, uint64_t nowUs)
{
    return r && r->initialized && nowUs >= r->receivedUs && nowUs >= r->sourceLocalUs &&
           nowUs - r->receivedUs <= (uint64_t)r->reference.leaseMs * 1000 &&
           nowUs - r->sourceLocalUs <= (uint64_t)r->reference.leaseMs * 1000;
}

// Preserve the cadence after small delays; skip missed slots without bursts.
bool df3MlrsSnapshotDue(uint32_t now, uint32_t *next)
{
    const uint32_t elapsed = now - *next;
    if (elapsed >= 0x80000000u) return false;
    *next += (elapsed / DF3_MLRS_SNAPSHOT_PERIOD_US + 1) * DF3_MLRS_SNAPSHOT_PERIOD_US;
    return true;
}

void df3MlrsSnapshotFrames(const uint8_t *snapshot, uint8_t *frames)
{
    for (unsigned part = 0; part < 2; ++part) {
        uint8_t *f = frames + part * 49;
        f[0] = 0xee;
        f[1] = 47;
        f[2] = 0xea;
        f[3] = 0xee;
        f[4] = 0xc8;
        f[5] = 1;
        f[6] = part;
        memcpy(f + 7, snapshot + 2, 8);
        memcpy(f + 15, snapshot + part * 33, 33);
        uint8_t crc = 0;
        for (unsigned i = 2; i < 48; ++i) {
            crc ^= f[i];
            for (unsigned bit = 0; bit < 8; ++bit) crc = (crc << 1) ^ (crc & 0x80 ? 0xd5 : 0);
        }
        f[48] = crc;
    }
}
