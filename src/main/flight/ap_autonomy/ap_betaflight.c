/* ArduPilot estimation/control behind the existing DF3 wire interface. */
#include "platform.h"
#ifdef USE_AP_AUTONOMY
#ifndef USE_DF3
#error "AP autonomy requires the DF3 reference and telemetry interface"
#endif

#include "ap_betaflight.h"
#include "ap_ekf.h"
#include "ap_control.h"
#include "runtime.h"
#include "ap_frame.h"
#include "ap_lifecycle.h"
#include "ap_mtf.h"
#include "ap_imu.h"
#include "backend/profile.h"
#include "backend/checkpoint.h"
#include "ap_worker.h"
#include "scheduler/scheduler.h"
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
#include "ap_blackbox.h"
#endif
#include "flight/df3/df3_betaflight.h"
#include "flight/df3/df3_frames.h"
#include "flight/df3/df3_flow.h"
#include "common/axis.h"
#ifndef SITL
#include "build/atomic.h"
#include "drivers/nvic.h"
#endif
#include "drivers/time.h"
#include "drivers/system.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "pg/df3.h"
#include "rx/rx.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#ifdef USE_ACCGYRO_BMI270
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#endif
#include "sensors/battery.h"
#include "sensors/opticalflow.h"
#include <math.h>
#ifdef SITL
#include "target/SITL/dfsim_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#endif
#include <string.h>

static bool booted, initializationAttempted, indoor, workerFault;
static ap_ekf_input_t sensorInput;
static ap_ekf_output_t navigation;
static ap_control_output actuator;
static df3Estimate_t estimate;
static df3ReferenceReceiver_t receiver;
static df3Reference_t reference;
static apLifecycle_t lifecycle;
static apLocalFrame_t localFrame;
static apMtfTiming_t mtfTiming;
static struct {
    uint64_t endUs;
    uint32_t intervalUs;
    int16_t rawX, rawY;
    uint8_t quality;
    bool pending;
} flowSample;
#ifndef SITL
static apMtfClock_t mtfClock;
#endif
static df3ControlOutput_t control;
static df3FaultDiagnostics_t diagnostics;
static df3GyroHistory_t gyroHistory;
static uint64_t imuUs, updateUs, rangeUs, flowUs, gpsUs;
static uint64_t attitudeUs, achievedThrustUs;
#ifdef SITL
static uint64_t logUs, lastBaroUs;
#endif
static float gyroFrd[3], attitudeFrd[4] = {1, 0, 0, 0};
static apImuDiagnostics_t imuDiagnostics;
static apImuGyro_t gyroIntegral;
static uint32_t gyroSampleUs, gyroSampleCount;
static float controllerForceFrd[3];
static uint64_t controllerForceUs;
static uint16_t stateEpoch, stateSequence;
static uint32_t controlSequence;
// Backend work may suspend. Only this completed snapshot is visible to the
// mixer, PID, telemetry and Blackbox; working state stays private to the job.
static struct {
    df3Estimate_t estimate;
    df3ControlOutput_t control;
    ap_control_output actuator;
    df3Reference_t reference;
    apLocalFrame_t frame;
    uint32_t filterFlags;
    uint16_t epoch;
    bool headingAligned;
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
    apBlackboxMetadata_t blackbox;
    apBlackboxVerticalDiagnostics_t verticalDiagnostics;
#endif
} published;
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
static bool verticalDiagnosticsEnabled;
static uint64_t consumedImuUs;
static apBlackboxAccounting_t imuAccounting;
static bool imuAccountingDiscontinuity;
static uint16_t accountingPositionReset;

static uint32_t sampleImuHardwareFlags(uint32_t nowUs, bool enabled)
{
#ifdef USE_ACCGYRO_BMI270
    static uint32_t lastRequestUs;
    uint32_t flags = 0, observationUs;
    uint8_t axes;
    // Consume even when logging is disabled so an unread result cannot block
    // future requests. This is a sparse observation, not part of the IMU batch.
    if (bmi270GetAccSaturation(acc.dev.gyro, &axes, &observationUs) &&
        enabled && verticalDiagnosticsEnabled && (uint32_t)(nowUs - observationUs) < 50000) {
        flags = AP_BLACKBOX_IMU_SATURATION_VALID | (uint32_t)axes * AP_BLACKBOX_IMU_SATURATION_X;
    }
    if (enabled && (uint32_t)(nowUs - lastRequestUs) >= 50000) {
        lastRequestUs = nowUs;
        bmi270RequestAccSaturation(acc.dev.gyro);
    }
    return flags;
#else
    UNUSED(nowUs);
    UNUSED(enabled);
    return 0;
#endif
}
#endif
static void runAutonomy(void);
static bool lowerLimited, upperLimited, attitudeLimited;
static float achievedThrust;
static bool achievedThrustValid;
static float groundRange;
static bool originEstablished, originFrozen, wasArmed, wasHeadingAligned, landed = true;
static struct {
    uint8_t payload[DF3_REFERENCE_BYTES];
    uint32_t timeUs, consumed;
    volatile uint32_t generation;
} mailbox;

uint64_t ap_runtime_time_us(void)
{
#ifdef SITL
    return micros64();
#else
    /* Calls originate in the cooperative scheduler, never the UART ISR. */
    static uint32_t previous;
    static uint64_t extended;
    const uint32_t current = micros();
    extended += (uint32_t)(current - previous);
    previous = current;
    return extended;
#endif
}

__attribute__((noreturn)) void ap_runtime_panic(const char *message)
{
#ifdef SITL
    fprintf(stderr, "AP autonomy fatal error: %s\n", message);
    abort();
#else
    (void)message;
    systemReset();
    while (true) {}
#endif
}

static void configureAssist(void)
{
    if (isModeActivationConditionPresent(BOXFLIGHTASSIST)) {
        return;
    }
    const modeActivationCondition_t empty = {0};
    for (unsigned i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; ++i) {
        modeActivationCondition_t *slot = modeActivationConditionsMutable(i);
        if (!memcmp(slot, &empty, sizeof(empty))) {
            *slot = (modeActivationCondition_t){.modeId = BOXFLIGHTASSIST, .auxChannelIndex = 2,
                .range = {.startStep = 32, .endStep = 48}, .modeLogic = MODELOGIC_OR};
            analyzeModeActivationConditions();
            return;
        }
    }
}

void df3BetaflightInit(void)
{
    if (initializationAttempted) {
        return;
    }
    initializationAttempted = true;
#ifdef SITL
    const char *mode = getenv("DFSIM_AP_INDOOR");
    indoor = !mode || strcmp(mode, "0") != 0;
#else
    /* Hardware qualification starts with flow/range and local BF heading.
     * GPS sensor delivery and in-flight source switching are not yet ported. */
    indoor = true;
#endif
    if (!ap_ekf_init(250, indoor) || !ap_control_init(.35f)) {
#ifdef SITL
        fprintf(stderr, "ArduPilot backend initialization failed\n");
#endif
        setArmingDisabled(ARMING_DISABLED_REBOOT_REQUIRED);
        return;
    }
    df3ReferenceReset(&receiver);
    df3GyroHistoryReset(&gyroHistory);
    for (unsigned i = 0; i < 3; ++i) {
        sensorInput.sensor_position_body_m[i] = .001f * df3FlowConfig()->sensorOffset[i];
    }
    actuator.hover_thrust = .35f;
    estimate.x[DF3_Q] = 1;
    configureAssist();
    apWorkerInit(runAutonomy);
    booted = true;
#ifdef SITL
    fprintf(stderr, "AP_AUTONOMY EKF3/AC_PosControl/AC_AttitudeControl, %s aiding, 250Hz host\n",
            indoor ? "flow/range" : "GPS/barometer");
    fprintf(stderr, "AP_STATE_HEADER,time_us,initialized,healthy,filter_status,armed,selected,permit,control_mode,ref_valid,p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z,q_w,q_x,q_y,q_z,ref_p_x,ref_p_y,ref_p_z,ref_v_x,ref_v_y,ref_v_z,ref_a_x,ref_a_y,ref_a_z,ref_yaw,rate_x,rate_y,rate_z,thrust,hover,accel_p,accel_i,accel_d,accel_ff,vertical_rate,control_vertical_rate,bias_z,height_innovation,height_test_ratio,height_acceptance_age_ms,vertical_degraded,terrain_down,terrain_transition,terrain_expired,terrain_age_ms\n");
#endif
}

// Keep the uncommon outage reset shared across the acquisition paths.
static __attribute__((noinline)) void resetImuIntegral(void)
{
    apImuGyroReset(&gyroIntegral);
    sensorInput.dt_s = 0;
    memset(sensorInput.delta_velocity_mps, 0, sizeof(sensorInput.delta_velocity_mps));
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
    // A callback can run while the backend is suspended. Apply this diagnostic
    // discontinuity at the next input snapshot, not to the job already running.
    imuAccountingDiscontinuity = true;
#endif
}

static bool takeImuInput(ap_ekf_input_t *sample)
{
    *sample = sensorInput;
    if (!(sensorInput.dt_s > 0 && gyroIntegral.dtS > 0)) {
        // A delayed callback must not discard the other sensor's integral.
        sample->dt_s = 0;
        return false;
    }
    sample->delta_angle_dt_s = apImuGyroTake(&gyroIntegral, sample->delta_angle_rad);
    sensorInput.dt_s = 0;
    sensorInput.fresh = 0;
    memset(sensorInput.delta_velocity_mps, 0, sizeof(sensorInput.delta_velocity_mps));
    return true;
}

void apAutonomyGyroSample(const struct gyroDev_s *dev)
{
    if (!booted) {
        return;
    }
    uint32_t sampleUs;
    if (dev->gyroHasSampleTiming) {
        if (dev->gyroSampleCount == gyroSampleCount) {
            return;
        }
        if (gyroSampleCount && (uint32_t)(dev->gyroSampleCount - gyroSampleCount) > 1) {
            ++imuDiagnostics.gap_count;
        }
        gyroSampleCount = dev->gyroSampleCount;
        sampleUs = dev->gyroSampleTimeUs;
    } else {
        // Drivers without captured metadata (including SITL) use receipt time.
        sampleUs = micros();
    }
    const uint32_t elapsedUs = sampleUs - gyroSampleUs;
    gyroSampleUs = sampleUs;
    if (!elapsedUs) {
        return;
    }
    if (elapsedUs > 10000) {
        if (gyroIntegral.initialized) {
            ++imuDiagnostics.gap_count;
        }
        resetImuIntegral();
        return;
    }
    float rateFrd[3];
    df3NativeVectorToFrd(dev->gyroADC, dev->scale * .01745329252f, rateFrd);
    apImuGyroPush(&gyroIntegral, rateFrd, elapsedUs * 1e-6f);
}

void df3BetaflightAccelerometer(void)
{
    if (!booted) {
        return;
    }
    const uint64_t now = ap_runtime_time_us();
    const uint32_t ageUs = (uint32_t)now - acc.dev.sampleTimeUs;
    if (ageUs > 10000) {
        ++imuDiagnostics.gap_count;
        resetImuIntegral();
        return;
    }
    const uint64_t sampleUs = now - ageUs;
    const float nativeGyro[3] = {gyroGetFilteredDownsampled(X), gyroGetFilteredDownsampled(Y), gyroGetFilteredDownsampled(Z)};
    const float nativeAccel[3] = {acc.accADC[X], acc.accADC[Y], acc.accADC[Z]};
    float force[3];
    df3NativeVectorToFrd(nativeGyro, .01745329252f, gyroFrd);
    df3NativeVectorToFrd(nativeAccel, 9.80665f * acc.dev.acc_1G_rec, force);
    if (acc.controllerAccelValid) {
        df3NativeVectorToFrd(acc.controllerAccel, 9.80665f * acc.dev.acc_1G_rec, controllerForceFrd);
        controllerForceUs = sampleUs;
    } else {
        controllerForceUs = 0;
    }
    memcpy(imuDiagnostics.force_mps2, force, sizeof(force));
    imuDiagnostics.sample_count += acc.dev.sampleCount ? acc.dev.sampleCount : 1;
    imuDiagnostics.clip_count += acc.dev.sampleClips;
    // The gyro value is the latest BF filtered rate, acquired separately from
    // the cached accelerometer batch. Do not backdate the flow gyro history.
    df3GyroHistoryPush(&gyroHistory, now, gyroFrd);
    const uint32_t elapsedUs = sampleUs - imuUs;
    // A driver's batch is the mean of actual acquired samples. Its duration
    // is independent of ACC task jitter; never integrate repeated DMA data.
    const float intervalUs = acc.dev.sampleCount ? acc.dev.sampleIntervalUs : elapsedUs;
    imuDiagnostics.last_interval_us = intervalUs;
    if (imuUs && sampleUs > imuUs && elapsedUs <= 10000 && intervalUs && intervalUs <= 10000) {
        if (acc.dev.sampleCount && elapsedUs > intervalUs + 1250) {
            ++imuDiagnostics.gap_count;
        }
        const float dt = intervalUs * 1e-6f;
        sensorInput.dt_s += dt;
        for (unsigned i = 0; i < 3; ++i) {
            sensorInput.delta_velocity_mps[i] += force[i] * dt;
        }
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
        // imuUs also advances after rejected intervals; record only batches
        // that actually contributed to the input being diagnosed.
        consumedImuUs = sampleUs;
#endif
    } else if (imuUs) {
        ++imuDiagnostics.gap_count;
        resetImuIntegral();
    }
    imuUs = sampleUs;
}

void apAutonomyImuDiagnostics(apImuDiagnostics_t *output)
{
    *output = imuDiagnostics;
}

void df3BetaflightAttitude(void)
{
    if (!booted) {
        return;
    }
    quaternion_t q;
    getQuaternion(&q);
    const float nativeQ[4] = {q.w, q.x, q.y, q.z};
    df3NativeQuaternionToFrd(nativeQ, attitudeFrd);
    attitudeUs = ap_runtime_time_us();
    // BF resets yaw at arm. Only the pre-arm heading bootstraps EKF3; subsequent
    // propagation remains in EKF3's continuous frame.
    if (indoor && !ARMING_FLAG(ARMED) && !published.headingAligned) {
        sensorInput.heading_rad = atan2f(2 * (attitudeFrd[0] * attitudeFrd[3] + attitudeFrd[1] * attitudeFrd[2]),
                                        1 - 2 * (attitudeFrd[2] * attitudeFrd[2] + attitudeFrd[3] * attitudeFrd[3]));
        sensorInput.heading_error_rad = .05f;
        sensorInput.heading_time_ms = attitudeUs / 1000;
        sensorInput.fresh |= AP_EKF_HEADING;
    }
}
void df3BetaflightRange(void) {}
void df3BetaflightFlow(void) {}

static uint32_t read32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

void df3BetaflightMtfFrame(const uint8_t p[20], uint8_t sequence)
{
    if (!booted) {
        return;
    }
    const uint32_t sourceMs = read32(p);
    const apMtfWindow_t window = apMtfWindow(&mtfTiming, sourceMs, sequence);
    if (window.duplicate) {
        return;
    }
    const uint64_t now = ap_runtime_time_us();
#ifdef SITL
    const uint64_t endUs = df3SitlSensorTimeUs(sourceMs);
#else
    const uint64_t endUs = apMtfTimeUs(&mtfClock, sourceMs, now);
#endif
    if (endUs > now || now - endUs > 100000) {
        return;
    }
    const float range = read32(p + 4) * .001f;
    if (p[10] && range >= .02f && range <= 12) {
        sensorInput.range_m = range;
        sensorInput.fresh |= AP_EKF_RANGE;
        rangeUs = endUs;
        if (!groundRange) {
            groundRange = range;
        }
    }
    flowSample.pending = window.valid && endUs >= window.intervalUs && p[17] && p[16];
    if (flowSample.pending) {
        flowSample.endUs = endUs;
        flowSample.intervalUs = window.intervalUs;
        flowSample.rawX = (int16_t)((uint16_t)p[12] | (uint16_t)p[13] << 8);
        flowSample.rawY = (int16_t)((uint16_t)p[14] | (uint16_t)p[15] << 8);
        flowSample.quality = p[16];
    }
}

static void consumeFlow(uint64_t now)
{
    if (!flowSample.pending) {
        return;
    }
    if (now < flowSample.endUs || now - flowSample.endUs > 100000) {
        flowSample.pending = false;
        return;
    }
    float averaged[3];
    /* The parser can run between IMU samples. Wait for measured gyro coverage
     * through the acquisition window instead of projecting the last rate. */
    if (!df3GyroHistoryAverage(&gyroHistory, flowSample.endUs - flowSample.intervalUs,
                              flowSample.endUs, averaged)) {
        return;
    }
    apMtfFlow(flowSample.rawX, flowSample.rawY, opticalflowConfig()->opticalflow_align,
              .001f * df3FlowConfig()->rotationScale, averaged, sensorInput.flow_rad_s);
    memcpy(sensorInput.flow_gyro_rad_s, averaged, sizeof(sensorInput.flow_gyro_rad_s));
    sensorInput.flow_quality = flowSample.quality;
    sensorInput.flow_time_ms = (flowSample.endUs - flowSample.intervalUs / 2) / 1000;
    sensorInput.fresh |= AP_EKF_FLOW;
    flowUs = flowSample.endUs;
    flowSample.pending = false;
}

#ifdef SITL
void apAutonomySitlAiding(const struct dfsim_input_v6_s *input)
{
    if (!booted) {
        return;
    }
    const uint64_t now = ap_runtime_time_us();
    if (!lastBaroUs || now - lastBaroUs >= 20000) {
        sensorInput.baro_altitude_m = 44330.0f * (1 - powf((float)input->base.serial.battery.base.pressurePa / 101325.0f, .19029495f));
        sensorInput.fresh |= AP_EKF_BARO;
        lastBaroUs = now;
    }
    if (input->sensorFlags & 1) {
        sensorInput.latitude_e7 = input->latitudeE7;
        sensorInput.longitude_e7 = input->longitudeE7;
        sensorInput.altitude_cm = input->altitudeCm;
        memcpy(sensorInput.gps_velocity_ned_mps, input->velocityNED, sizeof(input->velocityNED));
        sensorInput.gps_horizontal_accuracy_m = input->horizontalAccuracyM;
        sensorInput.gps_vertical_accuracy_m = input->verticalAccuracyM;
        sensorInput.gps_speed_accuracy_mps = input->speedAccuracyMps;
        sensorInput.gps_lag_s = (input->base.serial.battery.base.sampleUs - input->gpsSampleUs) * 1e-6f;
        sensorInput.gps_satellites = input->satellites;
        sensorInput.gps_fix = input->fixType;
        sensorInput.fresh |= AP_EKF_GPS;
        gpsUs = now;
    }
    if (input->sensorFlags & 2) {
        memcpy(sensorInput.magnetic_field_mgauss, input->magneticFieldBodyMGauss, sizeof(sensorInput.magnetic_field_mgauss));
        sensorInput.fresh |= AP_EKF_COMPASS;
    }
}
#endif

void df3BetaflightReferenceFrame(const uint8_t payload[DF3_REFERENCE_BYTES], uint32_t receivedUs)
{
    memcpy(mailbox.payload, payload, DF3_REFERENCE_BYTES);
    mailbox.timeUs = receivedUs;
    ++mailbox.generation;
}

static void consumeReference(uint64_t now, bool armed)
{
    uint8_t payload[DF3_REFERENCE_BYTES];
    uint32_t stamp = 0;
    bool fresh = false;
#ifndef SITL
    ATOMIC_BLOCK(NVIC_PRIO_MAX)
#endif
    {
        if (mailbox.generation != mailbox.consumed) {
            memcpy(payload, mailbox.payload, sizeof(payload));
            stamp = mailbox.timeUs;
            mailbox.consumed = mailbox.generation;
            fresh = true;
        }
    }
    const uint32_t age = (uint32_t)now - stamp;
    if (fresh && age <= 250000 && age <= now) {
        df3ReferenceAccept(&receiver, payload, now - age, armed);
    }
}

bool df3BetaflightReference(df3Reference_t *out)
{
    return df3ReferenceSample(&receiver, ap_runtime_time_us(), out);
}

bool df3BetaflightAssistSelected(void)
{
    return booted && IS_RC_MODE_ACTIVE(BOXFLIGHTASSIST) && !failsafeIsActive() &&
        !FLIGHT_MODE(GPS_RESCUE_MODE) && !FLIGHT_MODE(HEADFREE_MODE) &&
        !isFlipOverAfterCrashActive() && !isLaunchControlActive();
}

bool df3BetaflightAssistActive(void)
{
    return published.control.authority && ARMING_FLAG(ARMED) && rcData[THROTTLE] > rxConfig()->mincheck &&
        FLIGHT_MODE(ANGLE_MODE) && df3BetaflightAssistSelected();
}

const df3ControlOutput_t *df3BetaflightControl(void)
{
    static const df3ControlOutput_t stale = {.mode = DF3_CONTROL_FAULT, .authority = true};
    return ap_runtime_time_us() - updateUs <= 20000 ? &published.control : &stale;
}

float apAutonomyRateSetpoint(unsigned axis)
{
    if (axis > 2 || ap_runtime_time_us() - updateUs > 20000) {
        return 0;
    }
    return published.actuator.body_rate_target_rads[axis] * (axis == 0 ? 57.2957795f : -57.2957795f);
}

float apAutonomyMotorCommand(float thrust)
{
    return ap_control_thrust_to_actuator(thrust);
}

float apAutonomyMotorThrust(float command)
{
    return ap_control_actuator_to_thrust(command);
}

void apAutonomyMotorLimits(bool lower, bool upper, bool attitude, float achieved)
{
    lowerLimited = lower;
    upperLimited = upper;
    attitudeLimited = attitude;
    achievedThrust = achieved;
    achievedThrustValid = true;
    achievedThrustUs = ap_runtime_time_us();
}

#ifdef SITL
static void logState(uint64_t now, bool armed, bool selected, bool permit, bool haveReference)
{
    if (now - logUs < 20000) {
        return;
    }
    logUs = now;
    fprintf(stderr, "AP_STATE,%llu,%u,%u,%u,%u,%u,%u,%u,%u", (unsigned long long)now,
        navigation.initialized, navigation.healthy, navigation.filter_status, armed, selected, permit,
        control.mode, haveReference);
    for (unsigned i = 0; i < 9; ++i) { fprintf(stderr, ",%.7g", (double)estimate.x[i]); }
    for (unsigned i = 0; i < 4; ++i) { fprintf(stderr, ",%.7g", (double)estimate.x[DF3_Q + i]); }
    for (unsigned i = 0; i < 3; ++i) { fprintf(stderr, ",%.7g", (double)reference.position[i]); }
    for (unsigned i = 0; i < 3; ++i) { fprintf(stderr, ",%.7g", (double)reference.velocity[i]); }
    for (unsigned i = 0; i < 3; ++i) { fprintf(stderr, ",%.7g", (double)reference.acceleration[i]); }
    fprintf(stderr, ",%.7g", (double)reference.yaw);
    for (unsigned i = 0; i < 3; ++i) { fprintf(stderr, ",%.7g", (double)actuator.body_rate_target_rads[i]); }
    fprintf(stderr, ",%.7g,%.7g,%.7g,%.7g,%.7g,%.7g", (double)actuator.collective_thrust,
        (double)actuator.hover_thrust, (double)actuator.vertical_accel_p, (double)actuator.vertical_accel_i,
        (double)actuator.vertical_accel_d, (double)actuator.vertical_accel_ff);
    fprintf(stderr, ",%.7g,%.7g,%.7g,%.7g,%.7g,%u,%u,%.7g,%u,%u,%u\n",
        (double)navigation.vertical_position_rate_mps, (double)actuator.vertical_velocity_mps,
        (double)navigation.accel_bias_body_mps2[2], (double)navigation.height_innovation_m,
        (double)navigation.height_test_ratio, navigation.height_acceptance_age_ms, actuator.vertical_degraded,
        (double)navigation.terrain_offset_down_m, navigation.terrain_transition,
        navigation.terrain_expired, navigation.terrain_transition_age_ms);
}
#endif

bool apAutonomyReady(void)
{
    return !workerFault && (apWorkerBusy() || ap_runtime_time_us() - updateUs >= DF3_CONTROL_PERIOD_US);
}

void df3BetaflightTick(void)
{
    if (!booted) {
        df3BetaflightInit();
        if (!booted) {
            return;
        }
    }
    if (!apWorkerResume(schedulerTaskTimeAvailableUs())) {
        workerFault = true;
        published.estimate.valid = false;
        published.control = (df3ControlOutput_t){.mode = DF3_CONTROL_FAULT, .authority = true};
        setArmingDisabled(ARMING_DISABLED_REBOOT_REQUIRED);
    }
}

#ifndef SITL
// Bound the G4 adapter's flash footprint; raw IMU integration and the backend
// estimator/controller kernels are compiled independently.
__attribute__((optimize("Os")))
#endif
static void runAutonomy(void)
{
    const uint64_t now = ap_runtime_time_us();
    if (now - updateUs < DF3_CONTROL_PERIOD_US) {
        return;
    }
    const float dt = updateUs ? (now - updateUs) * 1e-6f : DF3_CONTROL_INITIAL_DT_S;
    const bool armed = ARMING_FLAG(ARMED);
    const bool selected = df3BetaflightAssistSelected();
    const bool permit = armed && selected && FLIGHT_MODE(ANGLE_MODE) && rcData[THROTTLE] > rxConfig()->mincheck;
    const bool nativeOverride = failsafeIsActive();
    if (wasArmed && !armed) {
        originEstablished = originFrozen = false;
        groundRange = 0;
        df3ReferenceReset(&receiver);
        mailbox.consumed = mailbox.generation;
    }
    // Freeze the last disarmed transform before BF's arm-time yaw reset can
    // change the SDK frame or reinterpret an already accepted waypoint.
    if (armed && originEstablished) {
        originFrozen = true;
    }
    wasArmed = armed;
    consumeReference(now, armed);
    const uint32_t flowStart = apProfileBegin(AP_PROF_FLOW);
    consumeFlow(now);
    apProfileEnd(AP_PROF_FLOW, flowStart);
    bool haveReference = df3ReferenceSample(&receiver, now, &reference);
    sensorInput.time_us = now;
    sensorInput.armed = armed;
    sensorInput.automatic_throttle = permit && !nativeOverride && control.authority &&
        control.mode != DF3_CONTROL_FAULT;
    sensorInput.takeoff_expected = armed && haveReference && reference.position[2] < estimate.x[DF3_P + 2] - .05f;
    // Snapshot before the first yield: callbacks collect the next batch while
    // this job runs, including timestamps which must not be compared to `now`.
    ap_ekf_input_t sensors;
    const bool haveImu = takeImuInput(&sensors);
    const uint64_t sampleImuUs = imuUs, sampleRangeUs = rangeUs, sampleFlowUs = flowUs;
    const uint64_t sampleGpsUs = gpsUs, sampleAttitudeUs = attitudeUs;
    const uint64_t sampleControllerForceUs = controllerForceUs;
    float sampleGyro[3], sampleAttitude[4], sampleControllerForce[3];
    memcpy(sampleGyro, gyroFrd, sizeof(sampleGyro));
    memcpy(sampleAttitude, attitudeFrd, sizeof(sampleAttitude));
    memcpy(sampleControllerForce, controllerForceFrd, sizeof(sampleControllerForce));
    ap_control_input input = {.time_us = now, .dt_s = dt,
        .throttle_lower_limited = lowerLimited, .throttle_upper_limited = upperLimited,
        .attitude_limited = attitudeLimited,
        .achieved_thrust_valid = achievedThrustValid && now - achievedThrustUs < 20000,
        .achieved_collective_thrust = achievedThrust};
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
    const bool logVertical = (df3BlackboxConfig()->axes & 2) != 0;
    if (logVertical != verticalDiagnosticsEnabled || imuAccountingDiscontinuity) {
        apBlackboxAccountingReset(&imuAccounting);
        imuAccountingDiscontinuity = false;
    }
    if (logVertical != verticalDiagnosticsEnabled) {
        ap_ekf_set_vertical_diagnostics(logVertical);
        verticalDiagnosticsEnabled = logVertical;
    }
    const uint32_t sampleHardwareFlags = sampleImuHardwareFlags((uint32_t)now, logVertical);
    apBlackboxAccountingObserveSaturation(&imuAccounting, sampleHardwareFlags);
    const uint64_t sampleConsumedImuUs = consumedImuUs;
    // Both values refer to the foreground-consumed driver snapshot, not a
    // newer ISR sum or a later Blackbox capture. Raw axes precede alignment.
    const bool sampleRawValid = acc.dev.sampleCount != 0;
    const uint32_t sampleRawSumZ = acc.dev.consumedSamples.axis[Z];
    const uint32_t sampleRawCount = acc.dev.consumedSamples.count;
#endif
    apAutonomyCheckpoint();
    const bool updated = haveImu && ap_ekf_update(&sensors, &navigation) != 0;
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
    ap_ekf_vertical_diagnostics_t verticalDiagnostics;
    if (logVertical) {
        memset(&verticalDiagnostics, 0, sizeof(verticalDiagnostics));
        const bool haveVerticalDiagnostics = updated &&
            ap_ekf_get_vertical_diagnostics(&verticalDiagnostics);
        if (navigation.position_d_reset_count != accountingPositionReset) {
            apBlackboxAccountingReset(&imuAccounting);
            accountingPositionReset = navigation.position_d_reset_count;
        }
        apBlackboxAccountingUpdate(&imuAccounting, &sensors,
            haveVerticalDiagnostics ? &verticalDiagnostics : NULL);
    }
#endif
    apAutonomyCheckpoint();
    if (updated) {
        estimate.timeUs = navigation.time_us;
        estimate.covarianceTimeUs = navigation.time_us;
        if (indoor && wasHeadingAligned && !navigation.heading_aligned && !armed) {
            // A new heading alignment establishes a new coordinate epoch.
            // Previously accepted targets cannot retain their old meaning.
            originEstablished = originFrozen = false;
            df3ReferenceReset(&receiver);
            mailbox.consumed = mailbox.generation;
            haveReference = false;
        }
        wasHeadingAligned = navigation.heading_aligned;
    }
    const bool imuFresh = sampleImuUs && now - sampleImuUs < 20000 &&
        sampleControllerForceUs && now - sampleControllerForceUs < 20000;
    const bool fusionFresh = navigation.time_us && now - navigation.time_us < 20000;
    const bool rangeFresh = sampleRangeUs && now - sampleRangeUs < 200000;
    const bool gpsFresh = sampleGpsUs && now - sampleGpsUs < 1000000;
    const bool lateralFresh = indoor ? sampleFlowUs && now - sampleFlowUs < 150000 : gpsFresh;
    const bool verticalFresh = indoor ? rangeFresh : gpsFresh;
    // nav_filter_status: attitude, horizontal/vertical velocity, height and
    // relative (indoor) or absolute (GPS) horizontal position must all be valid.
    const uint32_t required = 1u | 2u | 4u | 32u | (indoor ? 8u : 16u);
    estimate.valid = fusionFresh && navigation.healthy && imuFresh && (!indoor || navigation.heading_aligned) &&
        (navigation.filter_status & required) == required;
    // Range arrival and accepted aiding are different. Keep the raw freshness
    // gate too: the DAL can retain a previous "Good" range during an outage.
    if (indoor) {
        const bool terrainCoast = navigation.terrain_transition && !navigation.terrain_expired && rangeFresh &&
            navigation.height_acceptance_age_ms <= 1800;
        estimate.valid = estimate.valid && !navigation.terrain_expired &&
            (navigation.height_acceptance_age_ms <= 300 || terrainCoast) &&
            isfinite(navigation.height_innovation_m) && isfinite(navigation.height_test_ratio) &&
            isfinite(navigation.vertical_position_rate_mps);
    }
    if (!originFrozen && !armed && estimate.valid && sampleAttitudeUs && now - sampleAttitudeUs < 200000 &&
        (!indoor || rangeFresh)) {
        const float *q = navigation.quaternion;
        const float navYaw = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
        const float bfYaw = atan2f(2 * (sampleAttitude[0] * sampleAttitude[3] + sampleAttitude[1] * sampleAttitude[2]),
                                  1 - 2 * (sampleAttitude[2] * sampleAttitude[2] + sampleAttitude[3] * sampleAttitude[3]));
        const float cosTilt = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
        const float clearance = rangeFresh ? sensors.range_m * fmaxf(0, cosTilt) : 0;
        apFrameSet(&localFrame, navigation.position_ned_m, navYaw - bfYaw, clearance);
        if (!originEstablished) {
            if (++stateEpoch == 0) {
                ++stateEpoch;
            }
        }
        originEstablished = true;
    }
    if (originEstablished) {
        apFramePositionToLocal(&localFrame, navigation.position_ned_m, estimate.x + DF3_P);
        apFrameVectorToLocal(&localFrame, navigation.velocity_ned_mps, estimate.x + DF3_V);
        if (indoor && navigation.vertical_degraded) {
            // SDK trajectory handoff must use the same vertical feedback as
            // control. The local frame is yaw-only, so down is unchanged.
            estimate.x[DF3_V + 2] = navigation.vertical_position_rate_mps;
        }
        apFrameVectorToLocal(&localFrame, navigation.acceleration_ned_mps2, estimate.x + DF3_A);
        apFrameQuaternionToLocal(&localFrame, navigation.quaternion, estimate.x + DF3_Q);
    }
    estimate.valid = estimate.valid && originEstablished;
    estimate.verticalReferenceValid = estimate.valid && !navigation.terrain_transition;
    apAutonomyCheckpoint();
    if (!armed) {
        achievedThrustValid = false;
        lowerLimited = upperLimited = attitudeLimited = false;
    }
    if (!armed) {
        landed = true;
        if (rangeFresh) {
            groundRange = sensors.range_m;
        }
    } else if (rangeFresh) {
        if (sensors.range_m > groundRange + .05f) {
            landed = false;
        } else if (sensors.range_m < groundRange + .02f &&
                   fabsf(navigation.vertical_degraded ? navigation.vertical_position_rate_mps :
                         navigation.velocity_ned_mps[2]) < .15f && reference.velocity[2] >= 0) {
            landed = true;
        }
    }
    const apLifecycleInput_t policyInput = {.nowUs = now, .armed = armed, .selected = selected,
        .permit = permit, .nativeOverride = nativeOverride, .imuFresh = imuFresh,
        .verticalFresh = verticalFresh, .lateralFresh = lateralFresh, .estimate = &estimate,
        .reference = haveReference ? &reference : NULL};
    apLifecycleOutput_t policy;
    apLifecycleStep(&lifecycle, &policyInput, &policy);
    reference = policy.target;
    control = (df3ControlOutput_t){.mode = policy.mode, .authority = policy.authority};
    const bool active = policy.authority && policy.mode != DF3_CONTROL_FAULT;
    input.armed = active;
    input.landed = landed;
    input.lateral_control_enabled = policy.lateralControl;
    input.reference_yaw_rad = apFrameYawToNav(&localFrame, reference.yaw);
    input.position_ne_reset_count = navigation.position_ne_reset_count;
    input.position_d_reset_count = navigation.position_d_reset_count;
    input.vertical_position_rate_mps = navigation.vertical_position_rate_mps;
    input.vertical_degraded = indoor && navigation.vertical_degraded;
    input.terrain_transition = indoor && navigation.terrain_transition;
    memcpy(input.position_ned_m, navigation.position_ned_m, sizeof(input.position_ned_m));
    memcpy(input.velocity_ned_ms, navigation.velocity_ned_mps, sizeof(input.velocity_ned_ms));
    memcpy(input.quaternion_body_to_ned, navigation.quaternion, sizeof(input.quaternion_body_to_ned));
    for (unsigned i = 0; i < 3; ++i) {
        input.gyro_body_rads[i] = sampleGyro[i] - navigation.gyro_bias_body_rads[i];
    }
    // AP_AHRS_NavEKF3 uses filtered INS acceleration, subtracts the learned
    // body bias, then rotates into NED. EKF delta velocity stays unfiltered.
    for (unsigned i = 0; i < 3; ++i) {
        sampleControllerForce[i] -= navigation.accel_bias_body_mps2[i];
    }
    apFrameBodyToNav(navigation.quaternion, sampleControllerForce, input.specific_force_ned_mss);
    apFramePositionToNav(&localFrame, reference.position, input.reference_position_ned_m);
    apFrameVectorToNav(&localFrame, reference.velocity, input.reference_velocity_ned_ms);
    apFrameVectorToNav(&localFrame, reference.acceleration, input.reference_acceleration_ned_mss);
    apAutonomyCheckpoint();
    const uint32_t controlStart = apProfileBegin(AP_PROF_CONTROL);
    if (!navigation.initialized) {
        memset(&actuator, 0, sizeof(actuator));
    } else if (!ap_control_step(&input, &actuator)) {
        memset(&actuator, 0, sizeof(actuator));
        if (policy.authority) {
            apLifecycleFault(&lifecycle, &policy);
            control = (df3ControlOutput_t){.mode = policy.mode, .authority = policy.authority};
        }
    } else if (active) {
        control.throttle = actuator.collective_thrust;
        control.yawRateDeg = -actuator.body_rate_target_rads[2] * 57.2957795f;
        apFrameVectorToLocal(&localFrame, actuator.acceleration_target_ned_mss, control.acceleration);
    }
    apProfileEnd(AP_PROF_CONTROL, controlStart);
    apAutonomyCheckpoint();
    // No checkpoint in this commit. A timestamp remains the input acquisition
    // time: finishing a delayed job must not make old state appear fresh.
    if (ap_runtime_time_us() - now >= 20000) {
        estimate.valid = false;
        estimate.verticalReferenceValid = false;
        if (policy.authority) {
            apLifecycleFault(&lifecycle, &policy);
            control = (df3ControlOutput_t){.mode = policy.mode, .authority = policy.authority};
        }
    }
    const df3DiagnosticsInput_t checks = {.armed = armed, .selected = selected, .permit = permit,
        .imuFresh = imuFresh, .rangeFresh = verticalFresh, .flowFresh = lateralFresh,
        .fusionFresh = fusionFresh, .estimatorValid = estimate.valid, .referenceValid = haveReference,
        .controllerFault = control.mode == DF3_CONTROL_FAULT, .nativeFailsafe = nativeOverride,
        .rejected = receiver.rejected};
    df3DiagnosticsUpdate(&diagnostics, (uint32_t)now, &checks);
    published.estimate = estimate;
    published.control = control;
    published.actuator = actuator;
    published.reference = reference;
    published.frame = localFrame;
    published.filterFlags = navigation.filter_status;
    published.headingAligned = navigation.heading_aligned;
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
    published.blackbox = (apBlackboxMetadata_t){
        .controlUs = now,
        .referenceSourceUs = receiver.sourceLocalUs, .referenceReceiptUs = receiver.receivedUs,
        .referenceValid = haveReference,
        .verticalPositionRateMps = navigation.vertical_position_rate_mps,
        .ekfVerticalVelocityMps = navigation.velocity_ned_mps[2],
        .heightInnovationM = navigation.height_innovation_m,
        .heightTestRatio = navigation.height_test_ratio,
        .heightAcceptanceAgeMs = navigation.height_acceptance_age_ms,
        .rangeM = sensors.range_m,
        .imuSamples = sampleRawCount,
    };
    memcpy(published.blackbox.accelBiasBodyMps2, navigation.accel_bias_body_mps2,
           sizeof(published.blackbox.accelBiasBodyMps2));
    if (logVertical) {
        published.verticalDiagnostics = (apBlackboxVerticalDiagnostics_t){
            .imuTimeUs = sampleConsumedImuUs, .rangeTimeUs = sampleRangeUs,
            .inputDtS = sensors.dt_s, .fresh = sensors.fresh,
            .inputAngleDtS = sensors.delta_angle_dt_s,
            .controllerAccelDownMps2 = input.specific_force_ned_mss[2] + 9.80665f,
            .imuHardwareFlags = sampleHardwareFlags,
            .rawAccelSumZ = sampleRawSumZ, .rawSamplesValid = sampleRawValid,
            .accounting = imuAccounting,
            .ekf = verticalDiagnostics,
        };
        memcpy(published.verticalDiagnostics.deltaVelocityMps, sensors.delta_velocity_mps,
               sizeof(published.verticalDiagnostics.deltaVelocityMps));
        memcpy(published.verticalDiagnostics.deltaAngleRad, sensors.delta_angle_rad,
               sizeof(published.verticalDiagnostics.deltaAngleRad));
        published.blackbox.vertical = &published.verticalDiagnostics;
    }
#endif
    if (published.epoch != stateEpoch) {
        stateSequence = 0;
        published.epoch = stateEpoch;
    }
    updateUs = now;
    ++controlSequence;
#ifdef SITL
    logState(now, armed, selected, permit, haveReference);
#endif
}

#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
void df3BetaflightBlackbox(uint32_t nowUs, int32_t values[DF3_BLACKBOX_FIELD_COUNT])
{
    const uint64_t now = ap_runtime_time_us();
    apBlackboxMetadata_t metadata = published.blackbox;
    // Ages must use the timestamp written into the frame, not this later read.
    metadata.nowUs = now - (uint32_t)((uint32_t)now - nowUs);
    metadata.imuUs = imuUs;
    metadata.rangeUs = rangeUs;
    metadata.flowUs = flowUs;
    metadata.gpsUs = gpsUs;
    metadata.sample = controlSequence;
    metadata.filterFlags = published.filterFlags;
    metadata.controlMode = published.control.mode;
    metadata.armed = ARMING_FLAG(ARMED);
    metadata.authority = df3BetaflightAssistActive();
    apImuDiagnostics_t imu;
    apAutonomyImuDiagnostics(&imu);
    memcpy(metadata.prefilterForceBodyMps2, imu.force_mps2, sizeof(metadata.prefilterForceBodyMps2));
    metadata.imuClips = imu.clip_count;
    metadata.imuGaps = imu.gap_count;
    ap_control_output localActuator = published.actuator;
    apFrameVectorToLocal(&published.frame, published.actuator.acceleration_target_ned_mss, localActuator.acceleration_target_ned_mss);
    apBlackboxSnapshot(&published.estimate, &published.reference, &localActuator, &metadata, values);
}
#endif

const df3Estimate_t *df3BetaflightEstimate(void) { return &published.estimate; }
void df3BetaflightStatePayload(uint8_t payload[DF3_STATE_BYTES])
{
    df3StateEncode(&published.estimate, published.epoch, stateSequence++, ARMING_FLAG(ARMED), df3BetaflightAssistActive(), payload);
}
void df3BetaflightDiagnosticsPayload(uint8_t payload[DF3_DIAGNOSTICS_BYTES])
{
    df3DiagnosticsEncode(&diagnostics, ap_runtime_time_us() / 1000, payload);
}
void df3BetaflightTelemetryPoll(uint32_t now) { df3DiagnosticsTelemetryPoll(&diagnostics, now); }
void df3BetaflightTelemetryQueued(uint32_t now, bool replacing) { df3DiagnosticsTelemetryQueued(&diagnostics, now, replacing); }
void df3BetaflightUartSubmitted(uint32_t now, uint8_t type) { df3DiagnosticsUartSubmitted(&diagnostics, now, type); }
bool df3BetaflightFusionReady(void) { return false; }
void df3BetaflightFusionTick(void) {}
unsigned df3BetaflightFusionMinTimeUs(void) { return 1; }
unsigned df3BetaflightFusionBudgetUs(void) { return 1; }

// Legacy calibration is not used by the AP backend. Preserve the MSP symbols.
bool df3BetaflightCalibrationValid(float a0, float a1, float v0)
{
    return isfinite(a0) && isfinite(a1) && isfinite(v0);
}
float df3BetaflightCalibrationHover(float a0, float a1, float voltage)
{
    (void)a0; (void)a1; (void)voltage;
    return published.actuator.hover_thrust > 0 ? published.actuator.hover_thrust : .35f;
}
void df3BetaflightReloadCalibration(void) {}
void df3BetaflightReloadGains(void) {}
bool df3BetaflightAssistMappingReady(void)
{
    bool assigned = false;
    for (unsigned i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; ++i) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
        if (mac->auxChannelIndex != 2 || !IS_RANGE_USABLE(&mac->range) || mac->range.endStep <= 32 ||
            mac->range.startStep >= 48) {
            continue;
        }
        if (mac->modeId != BOXFLIGHTASSIST) {
            return false;
        }
        if (mac->range.startStep == 32 && mac->range.endStep == 48 && mac->modeLogic == MODELOGIC_OR &&
            !mac->linkedTo) {
            assigned = true;
        }
    }
    return assigned;
}
bool apAutonomyUsesLocalHeading(void) { return indoor; }
#endif
