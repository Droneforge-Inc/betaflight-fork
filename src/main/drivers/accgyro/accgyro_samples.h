#pragma once

#include <stdbool.h>
#include <stdint.h>

// At 800 Hz this retains 20 ms: longer than the autonomy bridge accepts.
// Only the controller needs sample order; EKF integration keeps using sums.
#define ACC_SAMPLE_HISTORY_LENGTH 16

// Publish a completed transfer separately from the DMA buffer, which may be
// overwritten by the next transfer while the foreground reads it.
typedef struct {
    int16_t raw[3];
    uint32_t timeUs, count;
} gyroSample_t;

static inline void gyroSamplePush(volatile gyroSample_t *sample, const uint8_t raw[6], uint32_t timeUs)
{
    for (unsigned axis = 0; axis < 3; ++axis) {
        sample->raw[axis] = (int16_t)((uint16_t)raw[2 * axis] | (uint16_t)raw[2 * axis + 1] << 8);
    }
    sample->timeUs = timeUs;
    ++sample->count;
}

static inline uint32_t gyroSampleRead(const volatile gyroSample_t *sample, int16_t raw[3], uint32_t *timeUs)
{
    uint32_t count;
    do {
        count = sample->count;
        for (unsigned axis = 0; axis < 3; ++axis) {
            raw[axis] = sample->raw[axis];
        }
        *timeUs = sample->timeUs;
    } while (count != sample->count);
    return count;
}

typedef struct accSampleHistory_s {
    int16_t raw[ACC_SAMPLE_HISTORY_LENGTH][3];
} accSampleHistory_t;

// Unsigned cumulative sums permit a reader to consume every sample without
// clearing an ISR-owned accumulator. Subtraction remains valid across wrap.
typedef struct accSampleSum_s {
    uint32_t axis[3];
    uint32_t clips;
    uint32_t timeUs;
    uint32_t count;
} accSampleSum_t;

static inline void accSampleSumPush(volatile accSampleSum_t *sum, const uint8_t raw[6], uint32_t timeUs)
{
    bool clipped = false;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const int16_t value = (int16_t)((uint16_t)raw[2 * axis] | (uint16_t)raw[2 * axis + 1] << 8);
        sum->axis[axis] += (uint32_t)(int32_t)value;
        clipped |= value >= 32700 || value <= -32700;
    }
    sum->clips += clipped;
    sum->timeUs = timeUs;
    // Publish last. The writer is an ISR, so it completes before the reader
    // can resume; the reader retries if the count changed while copying.
    ++sum->count;
}

static inline accSampleSum_t accSampleSumRead(const volatile accSampleSum_t *sum)
{
    accSampleSum_t snapshot;
    do {
        snapshot.count = sum->count;
        for (unsigned axis = 0; axis < 3; ++axis) {
            snapshot.axis[axis] = sum->axis[axis];
        }
        snapshot.clips = sum->clips;
        snapshot.timeUs = sum->timeUs;
    } while (snapshot.count != sum->count);
    return snapshot;
}

static inline void accSampleHistoryPush(volatile accSampleSum_t *sum,
    volatile accSampleHistory_t *history, const uint8_t raw[6], uint32_t timeUs)
{
    const unsigned index = sum->count % ACC_SAMPLE_HISTORY_LENGTH;
    for (unsigned axis = 0; axis < 3; ++axis) {
        history->raw[index][axis] = (int16_t)((uint16_t)raw[2 * axis] | (uint16_t)raw[2 * axis + 1] << 8);
    }
    // Publish the history together with the unchanged raw integral.
    accSampleSumPush(sum, raw, timeUs);
}

static inline accSampleSum_t accSampleHistoryRead(const volatile accSampleSum_t *sum,
    const volatile accSampleHistory_t *history, uint32_t previousCount,
    int16_t raw[ACC_SAMPLE_HISTORY_LENGTH][3])
{
    accSampleSum_t snapshot;
    do {
        snapshot = accSampleSumRead(sum);
        const uint32_t count = snapshot.count - previousCount;
        if (count <= ACC_SAMPLE_HISTORY_LENGTH) {
            for (uint32_t i = 0; i < count; ++i) {
                const unsigned index = (previousCount + i) % ACC_SAMPLE_HISTORY_LENGTH;
                for (unsigned axis = 0; axis < 3; ++axis) {
                    raw[i][axis] = history->raw[index][axis];
                }
            }
        }
    } while (snapshot.count != sum->count);
    return snapshot;
}

static inline uint32_t accSampleSumConsume(const accSampleSum_t *snapshot, accSampleSum_t *previous, float average[3])
{
    const uint32_t count = snapshot->count - previous->count;
    if (!count) {
        return 0;
    }
    const float scale = 1.0f / count;
    for (unsigned axis = 0; axis < 3; ++axis) {
        average[axis] = (int32_t)(snapshot->axis[axis] - previous->axis[axis]) * scale;
    }
    *previous = *snapshot;
    return count;
}
