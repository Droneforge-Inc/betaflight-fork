/* mLRS DF3 serial profile. Explicit little-endian fields; no packed structs. */
#pragma once
#include "df3_reference.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    DF3_MLRS_CAPABILITIES_BYTES = 12,
    DF3_MLRS_REFERENCE_BYTES = 32,
    DF3_MLRS_SNAPSHOT_BYTES = 66,
    DF3_MLRS_ACTIVE = 1,
    DF3_MLRS_AUTONOMY = 2,
    DF3_MLRS_ACK_VALID = 8,
    DF3_MLRS_PATH_READY = 16,
    DF3_MLRS_SNAPSHOT_PERIOD_US = 20000
};

static inline uint16_t df3ReadU16Le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline void df3WriteU16Le(uint8_t *p, uint16_t value)
{
    p[0] = value;
    p[1] = value >> 8;
}
static inline void df3WriteU32Le(uint8_t *p, uint32_t value)
{
    df3WriteU16Le(p, value);
    df3WriteU16Le(p + 2, value >> 16);
}

// Read-only MSP 0x30D4: no request payload; fixed versioned capability reply.
bool df3MlrsCapabilities(size_t requestLength, uint8_t reply[DF3_MLRS_CAPABILITIES_BYTES]);

// Convert the validated RF reference prefix into the unchanged legacy decoder.
bool df3MlrsReferenceDecode(const uint8_t wire[DF3_MLRS_REFERENCE_BYTES],
                            uint8_t legacy[DF3_REFERENCE_BYTES]);
// Repeated targets may accompany fresh RC, but must never extend target life.
bool df3MlrsReferenceAccept(df3ReferenceReceiver_t *receiver, uint8_t previous[DF3_MLRS_REFERENCE_BYTES],
                            const uint8_t wire[DF3_MLRS_REFERENCE_BYTES], uint64_t receivedUs, bool armed);
bool df3MlrsReferenceFresh(const df3ReferenceReceiver_t *receiver, uint64_t nowUs);
bool df3MlrsSnapshotDue(uint32_t now, uint32_t *next);
void df3MlrsSnapshotFrames(const uint8_t snapshot[DF3_MLRS_SNAPSHOT_BYTES], uint8_t frames[98]);
uint8_t df3MlrsAge10ms(uint64_t ageUs, bool valid);

#ifdef __cplusplus
}
#endif
