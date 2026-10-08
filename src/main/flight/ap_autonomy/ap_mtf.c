#include "ap_mtf.h"
#ifdef USE_AP_AUTONOMY

uint64_t apMtfTimeUs(apMtfClock_t *clock, uint32_t sourceMs, uint64_t receivedUs)
{
    const uint32_t deltaMs = sourceMs - clock->sourceMs;
    const bool reset = !clock->initialized || deltaMs > 1000;
    clock->extendedMs = reset ? sourceMs : clock->extendedMs + deltaMs;
    const int64_t observed = (int64_t)receivedUs - (int64_t)(clock->extendedMs * 1000);
    if (reset || observed < clock->minimumOffsetUs) {
        clock->minimumOffsetUs = observed;
    }
    clock->sourceMs = sourceMs;
    clock->initialized = true;
    return (uint64_t)((int64_t)(clock->extendedMs * 1000) + clock->minimumOffsetUs);
}

apMtfWindow_t apMtfWindow(apMtfTiming_t *timing, uint32_t timeMs, uint8_t sequence)
{
    const uint32_t deltaMs = timeMs - timing->timeMs;
    const uint8_t deltaSequence = sequence - timing->sequence;
    apMtfWindow_t window = {.duplicate = timing->initialized && !deltaMs && !deltaSequence};
    if (window.duplicate) {
        return window;
    }
    if (timing->initialized && deltaMs > 0 && deltaMs <= 250 && deltaSequence > 0 && deltaSequence <= 12) {
        window.intervalUs = deltaMs * 1000 / deltaSequence;
        window.valid = window.intervalUs >= 10000 && window.intervalUs <= 50000;
    }
    *timing = (apMtfTiming_t){.timeMs = timeMs, .sequence = sequence, .initialized = true};
    return window;
}

void apMtfFlow(int16_t rawX, int16_t rawY, uint8_t alignment, float rotationScale,
               const float gyroFrd[3], float flowRadS[2])
{
    float forward = .01f * rawX, right = .01f * rawY;
    const float x = forward, y = right;
    switch (alignment) {
    case 1: forward = -y; right = x; break;
    case 2: forward = -x; right = -y; break;
    case 3: forward = y; right = -x; break;
    default: break;
    }
    // EKF3 computes compensated LOS = -raw flow + measured gyro. Retain the
    // saved DF3 rotation coefficient without changing the gyro bias observation.
    flowRadS[0] = -right + (1 - rotationScale) * gyroFrd[0];
    flowRadS[1] = forward + (1 - rotationScale) * gyroFrd[1];
}
#endif
