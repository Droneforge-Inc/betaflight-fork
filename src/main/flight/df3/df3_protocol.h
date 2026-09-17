/* DF3 v1 payloads. Multibyte integers are big endian. Reference and state
 * positions are centimetres; velocity/acceleration are mm/s and mm/s^2;
 * angles are milliradians. Epoch, sequence and source time share one header. */
#pragma once
#include <stdint.h>

enum {
    DF3_OPTRANGE_CRSF_TYPE = 0xd3, // Existing sensor frame monitored by diagnostics.
    DF3_PROTOCOL_VERSION = 1,
    DF3_WIRE_VERSION = 0,
    DF3_WIRE_FLAGS = 1,
    DF3_WIRE_EPOCH = 2,
    DF3_WIRE_SEQUENCE = 4,
    DF3_WIRE_SOURCE_MS = 6,
    DF3_WIRE_HEADER_BYTES = 10,
    DF3_WIRE_POSITION = 10,
    DF3_WIRE_VELOCITY = 16,
    DF3_WIRE_ACCELERATION = 22,
    DF3_WIRE_ANGLES = 28,
    DF3_WIRE_REFERENCE_YAW = 28,
    DF3_WIRE_REFERENCE_LEASE = 30,
    DF3_REFERENCE_BYTES = 32,
    DF3_REFERENCE_CRSF_TYPE = 0xd5,
    DF3_REFERENCE_INACTIVE = 1,
    DF3_STATE_BYTES = 34,
    DF3_STATE_TYPE = 0xd6,
    DF3_STATE_VALID = 1,
    DF3_STATE_VERTICAL_VALID = 2,
    DF3_STATE_ARMED = 4,
    DF3_STATE_ASSIST = 8,
    DF3_DIAGNOSTICS_BYTES = 18,
    DF3_DIAGNOSTICS_TYPE = 0xd8,
    DF3_WIRE_DIAGNOSTICS_CURRENT = 10,
    DF3_WIRE_DIAGNOSTICS_LATCHED = 12,
    DF3_WIRE_DIAGNOSTICS_CHECKED = 14,
    DF3_WIRE_DIAGNOSTICS_LATCHED_CHECKED = 16,
    DF3_DIAGNOSTICS_ARMED = 1,
    DF3_DIAGNOSTICS_SELECTED = 2,
    DF3_DIAGNOSTICS_PERMIT = 4,
    DF3_DIAGNOSTICS_MONITORING = 8
};

static inline uint16_t df3ReadU16Be(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static inline int32_t df3ReadI16Be(const uint8_t *p)
{
    const uint16_t value = df3ReadU16Be(p);
    return value & 0x8000 ? (int32_t)value - 65536 : value;
}

static inline uint32_t df3ReadU32Be(const uint8_t *p)
{
    return ((uint32_t)df3ReadU16Be(p) << 16) | df3ReadU16Be(p + 2);
}

static inline void df3WriteU16Be(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}
