#pragma once

#include "flight/df3/df3_blackbox.h"
#include "flight/df3/df3_estimator.h"
#include "flight/df3/df3_reference.h"
#include "ap_control.h"
#include "ap_ekf.h"

enum { AP_BLACKBOX_SCHEMA = 5, AP_BLACKBOX_INVALID = -1000000001 };

// Sparse hardware observation, independent of the EKF/input-batch flags.
enum {
    AP_BLACKBOX_IMU_SATURATION_VALID = 1U << 12,
    AP_BLACKBOX_IMU_SATURATION_X = 1U << 13,
    AP_BLACKBOX_IMU_SATURATION_Y = 1U << 14,
    AP_BLACKBOX_IMU_SATURATION_Z = 1U << 15,
    AP_BLACKBOX_GENERATION_SHIFT = 20,
    AP_BLACKBOX_GENERATION_MASK = 0xff,
    AP_BLACKBOX_ACCOUNTING_VALID = 1U << 28,
    AP_BLACKBOX_RAW_SAMPLE_VALID = 1U << 29,
    AP_BLACKBOX_SUM_MASK = 0x3fffffff,
};

/* Diagnostics only: fixed-point sums survive omitted Blackbox snapshots.
 * No estimation or control output depends on this accounting. */
typedef struct {
    uint32_t sum[4];
    uint32_t saturationCounts;
    uint8_t generation;
    bool valid;
} apBlackboxAccounting_t;

enum {
    AP_BLACKBOX_SUM_INPUT_TIME,
    AP_BLACKBOX_SUM_INPUT_DV_Z,
    AP_BLACKBOX_SUM_OUTPUT_DV_Z,
    AP_BLACKBOX_SUM_OUTPUT_CORRECTION_Z,
};

void apBlackboxAccountingReset(apBlackboxAccounting_t *accounting);
void apBlackboxAccountingObserveSaturation(apBlackboxAccounting_t *accounting, uint32_t flags);
void apBlackboxAccountingUpdate(apBlackboxAccounting_t *accounting, const ap_ekf_input_t *input,
    const ap_ekf_vertical_diagnostics_t *diagnostics);

/* One completed EKF update's actual input and internal corrections. Input ages
 * are relative to metadata.controlUs, not the later Blackbox capture time. */
typedef struct {
    uint64_t imuTimeUs, rangeTimeUs;
    float inputDtS, inputAngleDtS, deltaVelocityMps[3], deltaAngleRad[3];
    float controllerAccelDownMps2;
    uint32_t fresh, imuHardwareFlags;
    uint32_t rawAccelSumZ;
    bool rawSamplesValid;
    apBlackboxAccounting_t accounting;
    ap_ekf_vertical_diagnostics_t ekf;
} apBlackboxVerticalDiagnostics_t;

/* Snapshot quantization (signed int32):
 *   position, velocity, acceleration, force and bias: SI x1000
 *   height test ratio: squared normalized gate statistic x1000 (>1000 fails)
 *   reference yaw, FRD body-rate target: rad or rad/s x1000
 *   body-to-NED quaternion [w,x,y,z]: x1e6
 *   normalized collective thrust, hover and acceleration PID terms: x1e6
 *   ages: microseconds, -1 means missing/future, capped at 1e9
 *   sample/counters: modulo 2^30; flags/modes/booleans: unscaled
 *   schema 3 input delta velocity/angle, height variance, internal corrections:
 *     SI x1e6; height observation: metres x1000; input duration: seconds x1e6
 *   schema 4 replaces the two ordinary output-correction fields with actual
 *     controller down acceleration (m/s² x1000, gravity removed) and raw body-Z
 *     population variance over consumed IMU samples ((m/s²)² x1e6).
 *   schema 5 replaces acceleration PID D/FF, latest prefilter Z, latest ACC
 *     interval, per-height-event velocity/bias corrections and raw variance:
 *     apRawAccelSumZ is the driver's cumulative signed raw sensor-Z ADC sum;
 *       BEFORE alignment/calibration, modulo 2^30. apImuSamples is its count
 *       at the SAME input snapshot, modulo 2^30. RAW_SAMPLE_VALID means a
 *       timestamped raw batch exists, not that sensor Z equals body Z. The
 *       saved mounting/trim must be used for a raw-to-body integral comparison.
 *     apInputTimeSumUs sums accepted EKF input durations, rounded to us/update;
 *     apInputDvSumZ sums input body-FRD Z delta velocity, before bias removal;
 *     apOutputDvSumZ sums navigation-down bias/gravity-corrected propagation;
 *     apOutputCorrectionSumZ sums additive ordinary-output velocity correction.
 *       These four sums wrap modulo 2^30; delta velocities use um/s. Signed
 *       differences must stay within +/-2^29 units between retained snapshots.
 *       Each increment is rounded independently (<=0.5 unit/update error).
 *     apInputAngleDtUs is the independent gyro integration duration.
 *     apImuSaturationCounts packs hardware observations modulo32768 in bits0..14
 *       and observations with ANY positive X/Y/Z bit modulo32768 in bits15..29.
 *       A poll counts positive once even when multiple axes saturate; the
 *       instantaneous hardware flags still identify individual observed axes.
 *       Zero positive counts cannot exclude clipping between sparse polls.
 *     apDiagFlags bits20..27 carry an accounting generation; bit28 validates
 *       the four integral sums and bit29 validates raw sum/count. Never
 *       difference across missing validity or generation changes. Diagnostics
 *       enable/disable, rejected input, input-stream discontinuity and EKF
 *       position reset start a new generation. Generation wraps after256
 *       resets; do not bridge long capture gaps. Driver sum/count do not reset.
 * Nonfinite floats use AP_BLACKBOX_INVALID; finite values saturate to ±1e9 to
 * keep Blackbox's signed delta encoding representable.
 *
 * State/reference PVA must share the same local-NED origin and orientation.
 * AP_ACCELERATION is physical acceleration, gravity removed. The vertical
 * PID P/I terms are positive-down; their sum subtracts from hover before AP's
 * angle boost and 2 Hz collective filter. AP_THRUST is after that filter and
 * before the Betaflight mixer/actuator curve. AP_ACTIVE reports the last AP
 * controller result, AP_AUTHORITY reports current Betaflight ownership.
 *
 * Sensor timestamps describe the latest accepted input, not fusion acceptance.
 * AP_HEIGHT_ACCEPTANCE_AGE is the age of EKF3's last height statistical pass/reset,
 * not a guarantee that height was fused in that cycle. AP_VERTICAL_RATE is
 * EKF3's complementary vertical derivative; AP_CONTROL_VERTICAL_RATE is the
 * actual controller velocity input. Bias and pre-filter specific force use
 * body FRD axes. AP_RANGE is the downward slant range, before tilt correction.
 * AP_EKF_VERTICAL_RATE preserves the ordinary EKF vertical velocity when the
 * published state velocity selects the complementary rate in degraded mode.
 * The caller supplies acquisition times where available; timestamps must use
 * the same monotonic clock as nowUs. Reference source time is mapped to that
 * clock by the existing reference receiver, independently of receipt time.
 * Schema 3 input/fusion ages use controlUs so they stay constant across repeated
 * snapshots. Height event fields are retained until the next successful fusion;
 * deduplicate apHeightFusionSeq when counting events. In schemas3/4 the height
 * corrections were per event; schema5 output corrections are cumulative instead.
 * apDiagFlags carries validity and event context.
 * A null vertical pointer marks the new fields unavailable; it never creates
 * plausible zero IMU increments. Remaining live force/ages retain their meaning.
 */
typedef struct {
    uint64_t nowUs, controlUs;
    uint64_t referenceSourceUs, referenceReceiptUs;
    uint64_t imuUs, rangeUs, flowUs, gpsUs;
    uint32_t sample, filterFlags;
    unsigned controlMode;
    bool referenceValid, armed, authority;
    float accelBiasBodyMps2[3], prefilterForceBodyMps2[3];
    float verticalPositionRateMps, heightInnovationM, heightTestRatio, rangeM;
    float ekfVerticalVelocityMps;
    uint32_t heightAcceptanceAgeMs; // UINT32_MAX means unavailable.
    uint32_t imuSamples, imuClips, imuGaps;
    const apBlackboxVerticalDiagnostics_t *vertical;
} apBlackboxMetadata_t;

/* Snapshot only; never feeds back into estimation or flight control. All
 * pointers are required, and the caller owns the destination storage. */
void apBlackboxSnapshot(const df3Estimate_t *estimate, const df3Reference_t *reference,
                       const ap_control_output *output, const apBlackboxMetadata_t *metadata,
                       int32_t values[DF3_BLACKBOX_FIELD_COUNT]);
