#include <stdint.h>
#if defined(USE_AP_AUTONOMY) && defined(USE_DF3_BLACKBOX)

#include "ap_blackbox.h"
#include <math.h>

// Keep per-field conversion paths shared; LTO duplicates them across the log.
static __attribute__((noinline)) int32_t quantize(float value, float scale)
{
    if (!isfinite(value)) {
        return AP_BLACKBOX_INVALID;
    }
    // G4 has single-precision hardware; logging must not pull software double
    // arithmetic into every recorded controller/state field.
    const float scaled = value * scale;
    if (scaled >= 1000000000.0f) {
        return 1000000000;
    }
    if (scaled <= -1000000000.0f) {
        return -1000000000;
    }
    return (int32_t)lroundf(scaled);
}

// Several discontinuity paths share this reset; keep LTO from duplicating it.
__attribute__((noinline)) void apBlackboxAccountingReset(apBlackboxAccounting_t *accounting)
{
    const uint8_t generation = accounting->generation + 1;
    *accounting = (apBlackboxAccounting_t){.generation = generation};
}

void apBlackboxAccountingObserveSaturation(apBlackboxAccounting_t *accounting, uint32_t flags)
{
    if (!(flags & AP_BLACKBOX_IMU_SATURATION_VALID)) {
        return;
    }
    const uint32_t counts = accounting->saturationCounts;
    const bool saturated = flags & (AP_BLACKBOX_IMU_SATURATION_X |
        AP_BLACKBOX_IMU_SATURATION_Y | AP_BLACKBOX_IMU_SATURATION_Z);
    // Independent 15-bit counters: all observations and any-positive polls.
    accounting->saturationCounts = ((counts + 1) & 0x7fffu) |
        ((((counts >> 15) + saturated) & 0x7fffu) << 15);
}

__attribute__((noinline)) void apBlackboxAccountingUpdate(apBlackboxAccounting_t *accounting, const ap_ekf_input_t *input,
    const ap_ekf_vertical_diagnostics_t *diagnostics)
{
    if (!diagnostics || !(diagnostics->flags & AP_EKF_DIAG_CORE_UPDATED) ||
        !(input->dt_s > 0 && input->dt_s <= .05f)) {
        goto invalid;
    }
    const float increments[] = {input->dt_s, input->delta_velocity_mps[2],
        diagnostics->output_delta_velocity_down_mps, diagnostics->output_velocity_delta_down_mps};
    // Validate the entire update before changing any sum. Unlike ordinary
    // snapshot fields, integrals must reject invalid/saturated values outright.
    for (unsigned i = 0; i < 4; ++i) {
        const float increment = increments[i];
        if (!(increment > -1000 && increment < 1000)) {
            goto invalid;
        }
    }
    // Unsigned addition preserves negative increments through modular wrap.
    for (unsigned i = 0; i < 4; ++i) {
        const uint32_t step = (uint32_t)(int32_t)lroundf(increments[i] * 1000000.0f);
        accounting->sum[i] = (accounting->sum[i] + step) & AP_BLACKBOX_SUM_MASK;
    }
    accounting->valid = true;
    return;

invalid:
    if (accounting->valid) {
        apBlackboxAccountingReset(accounting);
    }
}

static __attribute__((noinline)) int32_t age(uint64_t now, uint64_t time)
{
    if (!time || time > now) {
        return -1;
    }
    const uint64_t elapsed = now - time;
    return elapsed > 1000000000 ? 1000000000 : (int32_t)elapsed;
}

static int32_t heightAcceptanceAgeUs(uint32_t elapsedMs)
{
    if (elapsedMs == UINT32_MAX) {
        return -1;
    }
    return elapsedMs >= 1000000 ? 1000000000 : (int32_t)(elapsedMs * 1000);
}

static int32_t sampleAgeMs(uint64_t nowUs, uint32_t sampleMs)
{
    const int32_t elapsed = (int32_t)((uint32_t)(nowUs / 1000) - sampleMs);
    if (!sampleMs || elapsed < 0) {
        return -1;
    }
    return heightAcceptanceAgeUs((uint32_t)elapsed);
}

// This optional record is large; share its encoder instead of expanding it in
// the main snapshot. Flight estimation/control kernels keep their own flags.
static __attribute__((noinline, optimize("Os"))) void verticalSnapshot(const apBlackboxMetadata_t *metadata, int32_t *values)
{
    const apBlackboxVerticalDiagnostics_t *input = metadata->vertical;
    values[DF3_BB_AP_RAW_ACCEL_SUM_Z] = AP_BLACKBOX_INVALID;
    values[DF3_BB_AP_INPUT_TIME_SUM] = AP_BLACKBOX_INVALID;
    values[DF3_BB_AP_INPUT_DV_SUM_Z] = AP_BLACKBOX_INVALID;
    values[DF3_BB_AP_INPUT_ANGLE_DT] = AP_BLACKBOX_INVALID;
    if (!input) {
        for (unsigned field = DF3_BB_AP_INPUT_DT; field < DF3_BLACKBOX_FIELD_COUNT; ++field) {
            values[field] = AP_BLACKBOX_INVALID;
        }
        values[DF3_BB_AP_DIAGNOSTIC_FLAGS] = 0;
        return;
    }
    const ap_ekf_vertical_diagnostics_t *ekf = &input->ekf;
    const apBlackboxAccounting_t *accounting = &input->accounting;
    const bool fused = (ekf->flags & AP_EKF_DIAG_HEIGHT_EVENT) != 0;
    values[DF3_BB_AP_INPUT_DT] = quantize(input->inputDtS, 1000000);
    values[DF3_BB_AP_INPUT_ANGLE_DT] = quantize(input->inputAngleDtS, 1000000);
    values[DF3_BB_AP_INPUT_IMU_AGE] = age(metadata->controlUs, input->imuTimeUs);
    values[DF3_BB_AP_INPUT_RANGE_AGE] = age(metadata->controlUs, input->rangeTimeUs);
    values[DF3_BB_AP_INPUT_FRESH] = (int32_t)input->fresh;
    for (unsigned axis = 0; axis < 3; ++axis) {
        values[DF3_BB_AP_INPUT_DVX + axis] = quantize(input->deltaVelocityMps[axis], 1000000);
        values[DF3_BB_AP_INPUT_DAX + axis] = quantize(input->deltaAngleRad[axis], 1000000);
    }
    values[DF3_BB_AP_FUSION_AGE] = fused ? sampleAgeMs(metadata->controlUs, ekf->fusion_time_ms) : -1;
    values[DF3_BB_AP_HEIGHT_SAMPLE_AGE] = fused && (ekf->flags & AP_EKF_DIAG_HEIGHT_SAMPLE)
        ? sampleAgeMs(metadata->controlUs, ekf->height_sample_time_ms) : -1;
    values[DF3_BB_AP_HEIGHT_FUSION_SEQUENCE] = (int32_t)(ekf->height_fusion_sequence & 0x3fffffff);
    values[DF3_BB_AP_DIAGNOSTIC_FLAGS] = (int32_t)(ekf->flags | input->imuHardwareFlags |
        (uint32_t)accounting->generation << AP_BLACKBOX_GENERATION_SHIFT |
        (accounting->valid ? AP_BLACKBOX_ACCOUNTING_VALID : 0) |
        (input->rawSamplesValid ? AP_BLACKBOX_RAW_SAMPLE_VALID : 0));
    values[DF3_BB_AP_HEIGHT_OBSERVATION] = fused ? quantize(ekf->height_observation_down_m, 1000) : AP_BLACKBOX_INVALID;
    values[DF3_BB_AP_HEIGHT_VARIANCE] = fused ? quantize(ekf->height_variance_m2, 1000000) : AP_BLACKBOX_INVALID;
    if (input->rawSamplesValid) {
        values[DF3_BB_AP_RAW_ACCEL_SUM_Z] = input->rawAccelSumZ & AP_BLACKBOX_SUM_MASK;
    }
    values[DF3_BB_AP_OUTPUT_DV_SUM_Z] = AP_BLACKBOX_INVALID;
    values[DF3_BB_AP_OUTPUT_CORRECTION_SUM_Z] = AP_BLACKBOX_INVALID;
    if (accounting->valid) {
        values[DF3_BB_AP_INPUT_TIME_SUM] = accounting->sum[AP_BLACKBOX_SUM_INPUT_TIME];
        values[DF3_BB_AP_INPUT_DV_SUM_Z] = accounting->sum[AP_BLACKBOX_SUM_INPUT_DV_Z];
        values[DF3_BB_AP_OUTPUT_DV_SUM_Z] = accounting->sum[AP_BLACKBOX_SUM_OUTPUT_DV_Z];
        values[DF3_BB_AP_OUTPUT_CORRECTION_SUM_Z] = accounting->sum[AP_BLACKBOX_SUM_OUTPUT_CORRECTION_Z];
    }
    values[DF3_BB_AP_CONTROLLER_ACCEL] = quantize(input->controllerAccelDownMps2, 1000);
    values[DF3_BB_AP_IMU_SATURATION_COUNTS] = accounting->saturationCounts;
    values[DF3_BB_AP_OUTPUT_DELTA_VELOCITY] = quantize(ekf->output_delta_velocity_down_mps, 1000000);
    values[DF3_BB_AP_COMPLEMENTARY_ACCEL] = quantize(ekf->complementary_accel_correction_mps2, 1000000);
}

void apBlackboxSnapshot(const df3Estimate_t *estimate, const df3Reference_t *reference,
                       const ap_control_output *output, const apBlackboxMetadata_t *metadata,
                       int32_t values[DF3_BLACKBOX_FIELD_COUNT])
{
    // Each destination is written once; no zero-filled temporary snapshot.
    values[DF3_BB_AP_SCHEMA] = AP_BLACKBOX_SCHEMA;
    values[DF3_BB_AP_SAMPLE] = (int32_t)(metadata->sample & 0x3fffffff);
    values[DF3_BB_AP_CONTROL_AGE] = age(metadata->nowUs, metadata->controlUs);
    values[DF3_BB_AP_STATE_AGE] = age(metadata->nowUs, estimate->timeUs);
    values[DF3_BB_AP_FILTER_FLAGS] = (int32_t)metadata->filterFlags;
    values[DF3_BB_AP_ESTIMATE_VALID] = estimate->valid;
    values[DF3_BB_AP_MODE] = (int32_t)metadata->controlMode;
    values[DF3_BB_AP_ACTIVE] = output->active;
    values[DF3_BB_AP_ARMED] = metadata->armed;
    values[DF3_BB_AP_AUTHORITY] = metadata->authority;
    values[DF3_BB_AP_REFERENCE_VALID] = metadata->referenceValid;
    values[DF3_BB_AP_REFERENCE_EPOCH] = reference->epoch;
    values[DF3_BB_AP_REFERENCE_SEQUENCE] = reference->sequence;
    values[DF3_BB_AP_REFERENCE_AGE] = age(metadata->nowUs, metadata->referenceSourceUs);
    values[DF3_BB_AP_REFERENCE_RX_AGE] = age(metadata->nowUs, metadata->referenceReceiptUs);
    values[DF3_BB_AP_IMU_AGE] = age(metadata->nowUs, metadata->imuUs);
    values[DF3_BB_AP_RANGE_AGE] = age(metadata->nowUs, metadata->rangeUs);
    values[DF3_BB_AP_FLOW_AGE] = age(metadata->nowUs, metadata->flowUs);
    values[DF3_BB_AP_GPS_AGE] = age(metadata->nowUs, metadata->gpsUs);
    values[DF3_BB_AP_REFERENCE_YAW] = quantize(reference->yaw, 1000);
    values[DF3_BB_AP_THRUST] = quantize(output->collective_thrust, 1000000);
    values[DF3_BB_AP_HOVER] = quantize(output->hover_thrust, 1000000);
    values[DF3_BB_AP_ACCEL_PID_P] = quantize(output->vertical_accel_p, 1000000);
    values[DF3_BB_AP_ACCEL_PID_I] = quantize(output->vertical_accel_i, 1000000);
    values[DF3_BB_AP_VERTICAL_RATE] = quantize(metadata->verticalPositionRateMps, 1000);
    values[DF3_BB_AP_CONTROL_VERTICAL_RATE] = quantize(output->vertical_velocity_mps, 1000);
    values[DF3_BB_AP_HEIGHT_INNOVATION] = quantize(metadata->heightInnovationM, 1000);
    values[DF3_BB_AP_HEIGHT_TEST_RATIO] = quantize(metadata->heightTestRatio, 1000);
    values[DF3_BB_AP_HEIGHT_ACCEPTANCE_AGE] = heightAcceptanceAgeUs(metadata->heightAcceptanceAgeMs);
    values[DF3_BB_AP_RANGE] = quantize(metadata->rangeM, 1000);
    values[DF3_BB_AP_IMU_SAMPLES] = (int32_t)(metadata->imuSamples & 0x3fffffff);
    values[DF3_BB_AP_IMU_CLIPS] = (int32_t)(metadata->imuClips & 0x3fffffff);
    values[DF3_BB_AP_IMU_GAPS] = (int32_t)(metadata->imuGaps & 0x3fffffff);
    values[DF3_BB_AP_VERTICAL_DEGRADED] = output->vertical_degraded;
    values[DF3_BB_AP_EKF_VERTICAL_RATE] = quantize(metadata->ekfVerticalVelocityMps, 1000);
    // Each XYZ group in the field list is contiguous, independent of which
    // lateral/vertical fields Blackbox enables for transmission.
    for (unsigned axis = 0; axis < 3; ++axis) {
        values[DF3_BB_AP_POSITION_X + axis] = quantize(estimate->x[DF3_P + axis], 1000);
        values[DF3_BB_AP_VELOCITY_X + axis] = quantize(estimate->x[DF3_V + axis], 1000);
        values[DF3_BB_AP_ACCELERATION_X + axis] = quantize(estimate->x[DF3_A + axis], 1000);
        values[DF3_BB_AP_REFERENCE_P_X + axis] = quantize(reference->position[axis], 1000);
        values[DF3_BB_AP_REFERENCE_V_X + axis] = quantize(reference->velocity[axis], 1000);
        values[DF3_BB_AP_REFERENCE_A_X + axis] = quantize(reference->acceleration[axis], 1000);
        values[DF3_BB_AP_RATE_X + axis] = quantize(output->body_rate_target_rads[axis], 1000);
        values[DF3_BB_AP_ACCEL_TARGET_X + axis] = quantize(output->acceleration_target_ned_mss[axis], 1000);
        values[DF3_BB_AP_ACCEL_BIAS_X + axis] = quantize(metadata->accelBiasBodyMps2[axis], 1000);
        if (axis < 2) {
            values[DF3_BB_AP_PREFILTER_FORCE_X + axis] = quantize(metadata->prefilterForceBodyMps2[axis], 1000);
        }
    }
    for (unsigned axis = 0; axis < 4; ++axis) {
        values[DF3_BB_AP_QW + axis] = quantize(estimate->x[DF3_Q + axis], 1000000);
    }
    verticalSnapshot(metadata, values);
}
#endif
