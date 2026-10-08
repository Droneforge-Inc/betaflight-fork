#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float rateHz;
    float periodS;
    uint32_t count, timeUs;
    uint32_t windowCount, windowUs;
    bool initialized;
} apImuRate_t;

/* nominalHz must be positive. Counts and timestamps are cumulative acquisition
 * values, not the number or timing of scheduler callbacks. */
static inline void apImuRateReset(apImuRate_t *rate, float nominalHz)
{
    *rate = (apImuRate_t){.rateHz = nominalHz, .periodS = 1.0f / nominalHz};
}

/* Match AP_InertialSensor_Backend::_update_sensor_rate's one-second rate
 * observation and startup/normal weights. The caller selects convergence only
 * during its initial, disarmed period. A discontinuity restarts the observation
 * window; it must not turn a sensor outage into a slower learned sample rate. */
static inline float apImuRateUpdate(apImuRate_t *rate, uint32_t count, uint32_t timeUs,
    bool converging)
{
    const uint32_t samples = count - rate->count;
    if (rate->initialized && samples == 0) {
        return rate->periodS;
    }
    const uint32_t elapsedUs = timeUs - rate->timeUs;
    const float expected = (float)elapsedUs * (rate->rateHz * 1.0e-6f);
    if (!rate->initialized || elapsedUs == 0 || elapsedUs > 100000 ||
        samples < expected * 0.5f || samples > expected * 2.0f) {
        rate->windowCount = count;
        rate->windowUs = timeUs;
        rate->initialized = true;
    } else if (timeUs - rate->windowUs > 1000000) {
        float observedHz = (float)(count - rate->windowCount) * 1.0e6f /
            (float)(uint32_t)(timeUs - rate->windowUs);
        const float lower = rate->rateHz * (converging ? 0.5f : 0.95f);
        const float upper = rate->rateHz * (converging ? 2.0f : 1.05f);
        if (observedHz < lower) {
            observedHz = lower;
        } else if (observedHz > upper) {
            observedHz = upper;
        }
        const float weight = converging ? 0.2f : 0.02f;
        rate->rateHz += weight * (observedHz - rate->rateHz);
        rate->periodS = 1.0f / rate->rateHz;
        rate->windowCount = count;
        rate->windowUs = timeUs;
    }
    rate->count = count;
    rate->timeUs = timeUs;
    return rate->periodS;
}

typedef struct {
    float previousRadS[3];
    float lastAngleRad[3];
    float angleRad[3];
    float dtS;
    bool initialized;
} apImuGyro_t;

static inline void apImuGyroReset(apImuGyro_t *gyro)
{
    *gyro = (apImuGyro_t){0};
}

/* Calibrated body-FRD angular rate, before software filters. The first sample
 * seeds the trapezoid; reset on a stream discontinuity or calibration change.
 * Only the integration clock is validated here; sensor values must be finite. */
static inline bool apImuGyroPush(apImuGyro_t *gyro, const float radS[3], float dtS)
{
    if (!(dtS > 0.0f) || dtS > 0.1f) {
        apImuGyroReset(gyro);
        return false;
    }
    if (!gyro->initialized) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            gyro->previousRadS[axis] = radS[axis];
        }
        gyro->initialized = true;
        return false;
    }

    float delta[3], cone[3];
    for (unsigned axis = 0; axis < 3; ++axis) {
        delta[axis] = (radS[axis] + gyro->previousRadS[axis]) * (0.5f * dtS);
        cone[axis] = gyro->angleRad[axis] + gyro->lastAngleRad[axis] * (1.0f / 6.0f);
    }
    // AP's Tian et al. coning correction, including history across publication.
    const float correction[3] = {
        0.5f * (cone[1] * delta[2] - cone[2] * delta[1]),
        0.5f * (cone[2] * delta[0] - cone[0] * delta[2]),
        0.5f * (cone[0] * delta[1] - cone[1] * delta[0]),
    };
    for (unsigned axis = 0; axis < 3; ++axis) {
        gyro->angleRad[axis] += delta[axis] + correction[axis];
        gyro->lastAngleRad[axis] = delta[axis];
        gyro->previousRadS[axis] = radS[axis];
    }
    gyro->dtS += dtS;
    return true;
}

/* Returns the independently accumulated gyro interval, in seconds. Taking an
 * empty batch returns zero and preserves the sample/coning history. */
static inline float apImuGyroTake(apImuGyro_t *gyro, float angleRad[3])
{
    const float dtS = gyro->dtS;
    for (unsigned axis = 0; axis < 3; ++axis) {
        angleRad[axis] = gyro->angleRad[axis];
        gyro->angleRad[axis] = 0.0f;
    }
    gyro->dtS = 0.0f;
    return dtS;
}
