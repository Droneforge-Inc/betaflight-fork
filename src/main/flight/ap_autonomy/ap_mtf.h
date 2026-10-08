#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t timeMs;
    uint8_t sequence;
    bool initialized;
} apMtfTiming_t;

typedef struct {
    uint32_t intervalUs;
    bool duplicate;
    bool valid;
} apMtfWindow_t;

typedef struct {
    uint32_t sourceMs;
    uint64_t extendedMs;
    int64_t minimumOffsetUs;
    bool initialized;
} apMtfClock_t;

/* Map the sensor's wrapping clock into the FC clock. The minimum observed
 * offset rejects variable UART/scheduler delay; fixed transport delay remains. */
uint64_t apMtfTimeUs(apMtfClock_t *clock, uint32_t sourceMs, uint64_t receivedUs);

apMtfWindow_t apMtfWindow(apMtfTiming_t *timing, uint32_t timeMs, uint8_t sequence);

/* MicoLink cm/s at 1 m to AP angular flow. Alignment is BF CW0/90/180/270;
 * rotationScale affects gyro compensation, never translational flow scale. */
void apMtfFlow(int16_t rawX, int16_t rawY, uint8_t alignment, float rotationScale,
               const float gyroFrd[3], float flowRadS[2]);
