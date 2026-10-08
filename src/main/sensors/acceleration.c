/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/utils.h"

#include "config/feature.h"
#ifdef USE_AP_AUTONOMY
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "flight/ap_autonomy/ap_imu.h"
#endif

#include "sensors/acceleration_init.h"
#include "sensors/boardalignment.h"

#include "acceleration.h"
#ifdef USE_DF3
#include "flight/df3/df3_betaflight.h"
#endif

FAST_DATA_ZERO_INIT acc_t acc;                       // acc access functions

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC[X] -= accelerationTrims->raw[X];
    acc.accADC[Y] -= accelerationTrims->raw[Y];
    acc.accADC[Z] -= accelerationTrims->raw[Z];
}

#ifdef USE_AP_AUTONOMY
static biquadFilter_t controllerAccelFilter[XYZ_AXIS_COUNT];
static apImuRate_t accRate;
static uint32_t controllerSampleIntervalUs;
static uint32_t controllerLastSampleUs;
static bool controllerFilterInitialized;

static void updateControllerAcceleration(bool calibrated)
{
    acc.controllerAccelValid = false;
    const uint32_t elapsedUs = acc.dev.sampleTimeUs - controllerLastSampleUs;
    controllerLastSampleUs = acc.dev.sampleTimeUs;
    if (!calibrated || !acc.sampleRateHz ||
        (acc.dev.sampleCount && (!acc.dev.sampleHistoryValid || acc.dev.sampleCount > ACC_SAMPLE_HISTORY_LENGTH))) {
        // Lost history cannot be reconstructed from its mean. The EKF still
        // receives its original sum; controller validity exposes this gap.
        controllerFilterInitialized = false;
        return;
    }
    // Timestamped batches provide the sensor interval. Other drivers (including
    // SITL) deliver one fresh sample per callback, not necessarily at the ACC
    // task's advertised rate.
    const uint32_t intervalUs = acc.dev.sampleCount ? acc.dev.sampleIntervalUs / acc.dev.sampleCount :
        (elapsedUs && elapsedUs <= 10000 ? elapsedUs : lrintf(1e6f / acc.sampleRateHz));
    if (!intervalUs || intervalUs > 10000) {
        controllerFilterInitialized = false;
        return;
    }
    if (elapsedUs > 10000) {
        controllerFilterInitialized = false;
    }
    const bool intervalChanged = intervalUs != controllerSampleIntervalUs;
    controllerSampleIntervalUs = intervalUs;
    const unsigned count = acc.dev.sampleCount ? acc.dev.sampleCount : 1;
    for (unsigned sample = 0; sample < count; ++sample) {
        float value[XYZ_AXIS_COUNT];
        for (unsigned axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
            value[axis] = acc.dev.sampleCount ? acc.dev.sampleRawHistory[sample][axis] : acc.accADC[axis];
        }
        if (acc.dev.sampleCount) {
            if (acc.dev.accAlign == ALIGN_CUSTOM) {
                alignSensorViaMatrix(value, &acc.dev.rotationMatrix);
            } else {
                alignSensorViaRotation(value, acc.dev.accAlign);
            }
            for (unsigned axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
                value[axis] -= accelerationRuntime.accelerationTrims->raw[axis];
            }
        }
        for (unsigned axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
            biquadFilter_t *filter = &controllerAccelFilter[axis];
            if (!controllerFilterInitialized) {
                // Same 20 Hz two-pole Butterworth transfer function as AP's
                // INS_ACCEL_FILTER. Initialize at DC, not artificial zero g.
                biquadFilterInitLPF(filter, 20, intervalUs);
                filter->x1 = filter->x2 = filter->y1 = filter->y2 = value[axis];
            } else if (intervalChanged && sample == 0) {
                biquadFilterUpdateLPF(filter, 20, intervalUs);
            }
            acc.controllerAccel[axis] = biquadFilterApplyDF1(filter, value[axis]);
        }
        controllerFilterInitialized = true;
    }
    acc.controllerAccelValid = true;
}
#endif

void accUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }
    acc.isAccelUpdatedAtLeastOnce = true;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        const int16_t val =  acc.dev.ADCRaw[axis];
#ifdef USE_AP_AUTONOMY
        acc.accADC[axis] = acc.dev.sampleCount ? acc.dev.sampleRaw[axis] : val;
#else
        acc.accADC[axis] = val;
#endif
    }

    if (acc.dev.accAlign == ALIGN_CUSTOM) {
        alignSensorViaMatrix(acc.accADC, &acc.dev.rotationMatrix);
    } else {
        alignSensorViaRotation(acc.accADC, acc.dev.accAlign);
    }

    const bool calibrated = accIsCalibrationComplete();
    if (!calibrated) {
        performAcclerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    applyAccelerationTrims(accelerationRuntime.accelerationTrims);

#ifdef USE_AP_AUTONOMY
    // EKF delta velocity uses calibrated sensor samples before BF's attitude
    // output filter. Drivers without sample timestamps retain receipt timing.
    if (!acc.dev.sampleCount) {
        acc.dev.sampleTimeUs = micros();
    } else if (acc.sampleRateHz) {
        if (!accRate.rateHz) {
            apImuRateReset(&accRate, acc.sampleRateHz);
        }
        // Sensor ODR and FC clocks differ. Count captured samples, not ACC task
        // calls, and preserve the fractional interval in the raw delta velocity.
        const float periodS = apImuRateUpdate(&accRate, acc.dev.consumedSamples.count,
            acc.dev.sampleTimeUs, millis() < 30000 && !ARMING_FLAG(ARMED));
        acc.dev.sampleIntervalUs = acc.dev.sampleCount * (periodS * 1e6f);
    }
    updateControllerAcceleration(calibrated);
    if (calibrated) {
        df3BetaflightAccelerometer();
    }
#endif
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        const float val = acc.accADC[axis];
        acc.accADC[axis] = accelerationRuntime.accLpfCutHz ? pt2FilterApply(&accelerationRuntime.accFilter[axis], val) : val;
    }
#if defined(USE_DF3) && !defined(USE_AP_AUTONOMY)
    df3BetaflightAccelerometer();
#endif
}

#endif
