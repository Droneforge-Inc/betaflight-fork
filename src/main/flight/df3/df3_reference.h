/* Atomic versioned trajectory references; no allocation, no RC interpretation. */
#pragma once
#include "df3_protocol.h"
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// Reference validity is bounded by both receipt age and source-clock age.
enum { DF3_REFERENCE_MIN_LEASE_MS = 50, DF3_REFERENCE_MAX_LEASE_MS = 250, DF3_REFERENCE_MAX_SOURCE_LEAD_US = 250000 };
typedef struct {
    float position[3], velocity[3], acceleration[3], yaw;
    uint32_t sourceMs;
    uint16_t epoch, sequence, leaseMs;
    bool active;
} df3Reference_t;
typedef struct {
    df3Reference_t reference;
    uint64_t receivedUs, sourceLocalUs;
    uint32_t accepted, rejected;
    bool initialized;
} df3ReferenceReceiver_t;

void df3ReferenceReset(df3ReferenceReceiver_t *receiver);
bool df3ReferenceDecode(const uint8_t payload[DF3_REFERENCE_BYTES], df3Reference_t *out);
bool df3ReferenceAccept(df3ReferenceReceiver_t *receiver, const uint8_t payload[DF3_REFERENCE_BYTES],
                        uint64_t receivedUs, bool armed);
bool df3ReferenceSample(const df3ReferenceReceiver_t *receiver, uint64_t nowUs, df3Reference_t *out);
#ifdef __cplusplus
}
#endif
