/* Native sensor boundary and explicitly gated onboard flight assist. */
#include "platform.h"
#ifdef USE_DF3
#if !defined(USE_ACC) || !defined(USE_RANGEFINDER_OPTFLOW_MTF) || !defined(USE_OPTICALFLOW)
#error "DF3 requires accelerometer, MTF rangefinder, and optical flow"
#endif
#include "common/axis.h"
#include "df3_betaflight.h"
#include "df3_flow.h"
#include "df3_profile.h"
#ifdef USE_DF3_RESUMABLE
#include "df3_resumable.h"
static df3FusionJob_t fusionJob;
#endif
#ifdef USE_DF3_BUDGETED_WORKER
#if !defined(USE_DF3_RESUMABLE) || (!defined(SITL) && !defined(USE_DF3_BUDGET_CYCLES))
#error "Budgeted DF3 requires resumable work and hardware cycle timing"
#endif
#if defined(SITL) && defined(USE_DF3_JOSEPH_ASM)
#error "Host ARM emulation time is not an MCU deadline clock; use C for native timing"
#endif
#include "df3_budget.h"
#include "scheduler/scheduler.h"
#include "drivers/system.h"
#ifdef SITL
#include <time.h>
#endif
static df3BudgetWorker_t fusionWorker;
static df3BudgetTicks_t fusionClock(void *unused)
{
    (void)unused;
#ifdef SITL
    struct timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return (uint64_t)ts.tv_sec * 1000000000 + (uint64_t)ts.tv_nsec;
#else
    return getCycleCounter(); // Wall cycles include interrupt service during a slice.
#endif
}
#endif
#include "df3_frames.h"
#include "drivers/time.h"
#include "fc/rc_controls.h"
#include "fc/rc.h"
#include "fc/rc_modes.h"
#include "fc/core.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "pg/df3.h"
#include "rx/rx.h"
#include "sensors/acceleration.h"
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"
#endif
#include "sensors/gyro.h"
#include "sensors/opticalflow.h"
#include "sensors/rangefinder.h"
#include "sensors/battery.h"
#include <math.h>
#include <string.h>
#ifndef SITL
#include "build/atomic.h"
#include "drivers/nvic.h"
#endif
#ifdef SITL
#include <stdio.h>
#include <stdlib.h>
#include "target/SITL/dfsim_protocol.h"
static FILE *logFile;
#ifdef USE_DF3_SCHED_BENCH
#include <time.h>
#include "scheduler/scheduler.h"
static FILE *scheduleLog;
static uint64_t benchGyroCalls, benchPidCalls;
static uint32_t benchGyroGap, benchPidGap;
void df3BetaflightBenchTask(unsigned taskId, uint32_t intervalUs)
{
    if (taskId == TASK_GYRO) {
        if (benchGyroCalls++ && intervalUs > benchGyroGap) {
            benchGyroGap = intervalUs;
        }
    } else if (taskId == TASK_PID) {
        if (benchPidCalls++ && intervalUs > benchPidGap) {
            benchPidGap = intervalUs;
        }
    }
}
static uint64_t benchCpuNs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return (uint64_t)ts.tv_sec * 1000000000 + (uint64_t)ts.tv_nsec;
}
#endif
#endif

static df3Estimator_t estimator;
static df3GyroHistory_t gyroHistory;
#ifdef USE_DF3_MULTIRATE
static df3ImuReducer_t imuReducer;
#endif
static df3Estimate_t estimate;
static df3FlowResult_t lastFlow;
static df3ReferenceReceiver_t referenceReceiver;
static df3Control_t controller;
static df3ControlConfig_t controlConfig;
static df3ControlOutput_t controlOutput;
static uint64_t controlUs;
static df3FaultDiagnostics_t diagnostics;
static uint16_t stateEpoch, stateSequence;
static struct {
    uint8_t payload[DF3_REFERENCE_BYTES];
    uint32_t receivedUs;
    volatile uint32_t generation;
} referenceMailbox;
static uint32_t referenceGeneration;
static float lastGyro[3], lastQ[4], lastRange, lastStrength;
static uint64_t lastImuUs, lastAttitudeUs, lastRangeUs, lastGyroHistoryUs;
static bool booted, wasArmed, activationQueued;
static struct {
    uint64_t receivedUs, endUs;
    uint32_t generation, intervalUs, previousSourceMs, rangeMm;
    uint8_t sequence;
    bool timingValid, hasPrevious;
    uint8_t rangeStatus, flowStatus, quality, strength;
} mtf;
static uint32_t rangeGeneration, flowGeneration, flowAccepted, flowRejected;
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
static df3FusionTrace_t blackboxFusion;
static int32_t blackboxValues[DF3_BLACKBOX_FIELD_COUNT];
static uint64_t blackboxControlUs, blackboxRangeReceiptUs;
static uint32_t blackboxSequence, blackboxRangeMm;
static float blackboxForce[3], blackboxFlowClearance;
static uint8_t blackboxFlowQuality;

// Bounded integers keep previous-frame differences within signed 32 bits.
// The sentinel distinguishes unavailable/nonfinite data from a real zero.
static int32_t blackboxValue(float value, float scale)
{
    const float scaled = value * scale;
    return isfinite(scaled) ? lrintf(fmaxf(-1e9f, fminf(1e9f, scaled))) : -1000000001;
}

static int32_t blackboxAge(uint64_t now, uint64_t sample)
{
    return !sample || sample > now ? -1 : (int32_t)MIN(now - sample, 1000000000ULL);
}

static void captureBlackbox(uint64_t now, bool haveReference)
{
    if (blackboxConfig()->device == BLACKBOX_DEVICE_NONE) {
        return;
    }
    int32_t *v = blackboxValues;
    const df3ControlAxisTrace_t *c = &controlOutput.trace.axis[2];
    v[DF3_BB_SAMPLE] = (int32_t)(++blackboxSequence & 0x3fffffff);
    v[DF3_BB_FLAGS] = controlOutput.mode | (controlOutput.authority << 3) |
        (estimate.valid << 4) | (estimate.verticalReferenceValid << 5) | (haveReference << 6);
    v[DF3_BB_REFERENCE_AGE] = blackboxAge(now, referenceReceiver.receivedUs);
    for (unsigned i = 0; i < 3; ++i) {
        v[DF3_BB_REFERENCE_P + i] = blackboxValue(c->reference[i], 1000);
        v[DF3_BB_POSITION + i] = blackboxValue(estimate.x[3 * i + 2], 1000);
        v[DF3_BB_FORCE_X + i] = blackboxValue(blackboxForce[i], 1000);
        v[DF3_BB_BIAS_X + i] = blackboxValue(estimate.x[DF3_BA + i], 1000);
    }
    v[DF3_BB_PROJECTED_P] = blackboxValue(fusionJob.committed.x[2], 1000);
    v[DF3_BB_PROJECTED_V] = blackboxValue(fusionJob.committed.x[5], 1000);
    v[DF3_BB_FUSION_AGE] = blackboxAge(now, estimate.covarianceTimeUs);
    for (unsigned i = 0; i < 4; ++i) {
        v[DF3_BB_QW + i] = blackboxValue(estimate.x[DF3_Q + i], 1000000);
        v[DF3_BB_FEEDFORWARD + i] = blackboxValue(c->terms[i], 1000);
    }
    v[DF3_BB_IMU_AGE] = blackboxAge(now, lastImuUs);
    v[DF3_BB_RANGE_RAW] = (int32_t)MIN(blackboxRangeMm, 1000000000U);
    v[DF3_BB_RANGE_DOWN] = blackboxValue(lastRange, 1000);
    v[DF3_BB_RANGE_STRENGTH] = blackboxValue(lastStrength, 1);
    v[DF3_BB_RANGE_AGE] = blackboxAge(now, lastRangeUs);
    v[DF3_BB_RANGE_RECEIPT_AGE] = blackboxAge(now, blackboxRangeReceiptUs);
    v[DF3_BB_TERRAIN] = blackboxValue(estimate.terrainDown, 1000);
    for (unsigned i = 0; i < 2; ++i) {
        const df3FusionTraceSample_t *s = i ? &blackboxFusion.range : &blackboxFusion.imu;
        const unsigned first = i ? DF3_BB_RANGE_SEQUENCE : DF3_BB_IMU_SEQUENCE;
        v[first] = (int32_t)(s->sequence & 0x3fffffff);
        v[first + 1] = blackboxAge(now, s->sensorUs);
        v[first + 2] = blackboxValue(s->measurement[2], 1000);
        v[first + 3] = blackboxValue(s->innovation[2], 1000);
        v[first + 4] = blackboxValue(s->variance[2], i ? 1000000 : 1000);
        v[first + 5] = blackboxValue(s->deltaV[2], 1000000);
        v[first + 6] = blackboxValue(s->deltaA[2], 1000000);
        v[first + 7] = s->status;
    }
    v[DF3_BB_RANGE_FLAGS] = blackboxFusion.rangeFlags;
    v[DF3_BB_ACCEL_REQUESTED] = blackboxValue(c->requestedAccel, 1000);
    v[DF3_BB_ACCEL_APPLIED] = blackboxValue(controlOutput.acceleration[2], 1000);
    v[DF3_BB_INTEGRAL] = blackboxValue(c->integral, 1000);
    v[DF3_BB_HOVER] = blackboxValue(controlConfig.hoverThrottle, 10000);
    v[DF3_BB_THROTTLE_REQUESTED] = blackboxValue(controlOutput.trace.requestedThrottle, 10000);
    v[DF3_BB_THROTTLE_APPLIED] = blackboxValue(controlOutput.throttle, 10000);
    v[DF3_BB_INTEGRATION_HELD] = c->integrationHeld;
    v[DF3_BB_REFERENCE_SEQUENCE] = referenceReceiver.reference.sequence;
    v[DF3_BB_DIAGNOSTICS] = diagnostics.current;
    v[DF3_BB_LOG_DROPS] = (int32_t)MIN(blackboxGetDroppedBytes(), 1000000000U);
    v[DF3_BB_IMU_ROUGHNESS] = blackboxValue(blackboxFusion.imuRoughness, 1000000);
    v[DF3_BB_EPOCH] = stateEpoch;
    v[DF3_BB_QUEUE] = estimator.count + (fusionJob.busy ? fusionJob.count - fusionJob.index : 0);
    // X/Y control blocks share the Z semantics and use local-NED axes.
    const unsigned controlFields[2] = {DF3_BB_REFERENCE_P_X, DF3_BB_REFERENCE_P_Y};
    const unsigned imuFields[2] = {DF3_BB_IMU_MEASUREMENT_X, DF3_BB_IMU_MEASUREMENT_Y};
    const unsigned flowFields[2] = {DF3_BB_FLOW_MEASUREMENT_X, DF3_BB_FLOW_MEASUREMENT_Y};
    const unsigned rawFlowFields[2] = {DF3_BB_FLOW_RAW_X, DF3_BB_FLOW_RAW_Y};
    for (unsigned axis = 0; axis < 2; ++axis) {
        const unsigned first = controlFields[axis];
        const df3ControlAxisTrace_t *a = &controlOutput.trace.axis[axis];
        for (unsigned i = 0; i < 3; ++i) {
            v[first + i] = blackboxValue(a->reference[i], 1000);
            v[first + 3 + i] = blackboxValue(estimate.x[3 * i + axis], 1000);
        }
        for (unsigned i = 0; i < 2; ++i) {
            v[first + 6 + i] = blackboxValue(fusionJob.committed.x[3 * i + axis], 1000);
        }
        for (unsigned i = 0; i < 4; ++i) {
            v[first + 8 + i] = blackboxValue(a->terms[i], 1000);
        }
        v[first + 12] = blackboxValue(a->requestedAccel, 1000);
        v[first + 13] = blackboxValue(controlOutput.acceleration[axis], 1000);
        v[first + 14] = blackboxValue(a->integral, 1000);
        v[first + 15] = a->integrationHeld;
        for (unsigned kind = 0; kind < 2; ++kind) {
            const df3FusionTraceSample_t *s = kind ? &blackboxFusion.flow : &blackboxFusion.imu;
            const unsigned field = kind ? flowFields[axis] : imuFields[axis];
            v[field] = blackboxValue(s->measurement[axis], 1000);
            v[field + 1] = blackboxValue(s->innovation[axis], 1000);
            v[field + 2] = blackboxValue(s->variance[axis], kind ? 1000000 : 1000);
            v[field + 3] = blackboxValue(s->deltaV[axis], 1000000);
            v[field + 4] = blackboxValue(s->deltaA[axis], 1000000);
        }
        v[rawFlowFields[axis]] = blackboxValue(lastFlow.rawBodyVelocity[axis], 1000);
        v[rawFlowFields[axis] + 1] = blackboxValue(lastFlow.rotationCorrection[axis], 1000);
        v[rawFlowFields[axis] + 2] = blackboxValue(lastFlow.leverCorrection[axis], 1000);
    }
    v[DF3_BB_ROLL_REQUESTED] = blackboxValue(controlOutput.angleDeg[0], 1000);
    v[DF3_BB_PITCH_REQUESTED] = blackboxValue(controlOutput.angleDeg[1], 1000);
    v[DF3_BB_IMU_ROUGHNESS_XY] = blackboxValue(blackboxFusion.imuRoughnessXY, 1000000);
    v[DF3_BB_FLOW_SEQUENCE] = (int32_t)(blackboxFusion.flow.sequence & 0x3fffffff);
    v[DF3_BB_FLOW_FUSION_AGE] = blackboxAge(now, blackboxFusion.flow.sensorUs);
    v[DF3_BB_FLOW_STATUS] = blackboxFusion.flow.status;
    v[DF3_BB_FLOW_AGE] = blackboxAge(now, lastFlow.midpointUs);
    v[DF3_BB_FLOW_INTERVAL] = (int32_t)MIN(lastFlow.endUs - lastFlow.startUs, 1000000000U);
    v[DF3_BB_FLOW_QUALITY] = blackboxFlowQuality;
    v[DF3_BB_FLOW_CLEARANCE] = blackboxValue(blackboxFlowClearance, 1000);
    v[DF3_BB_FLOW_ACCEPTED] = (int32_t)(flowAccepted & 0x3fffffff);
    v[DF3_BB_FLOW_REJECTED] = (int32_t)(flowRejected & 0x3fffffff);
    v[DF3_BB_FLOW_GYRO_P] = blackboxValue(lastFlow.averageGyro[0], 1000000);
    v[DF3_BB_FLOW_GYRO_Q] = blackboxValue(lastFlow.averageGyro[1], 1000000);
    blackboxControlUs = now;
}

void df3BetaflightBlackbox(uint32_t nowUs, int32_t values[DF3_BLACKBOX_FIELD_COUNT])
{
    memcpy(values, blackboxValues, sizeof(blackboxValues));
    values[DF3_BB_AGE] = blackboxControlUs ?
        (int32_t)MIN((uint32_t)(nowUs - (uint32_t)blackboxControlUs), 1000000000U) : -1;
}
#endif
/* MTF-02 flight recordings exhibit a 10-count grid at 1 m, despite the
 * normalized cm/s wire units. Match that observed acquisition resolution. */
static float flowQuantizationCMPS = 10;

float df3BetaflightCalibrationHover(float a0, float a1, float voltage)
{
    return rcCalibrationCollective(a0 + a1 * voltage);
}

bool df3BetaflightCalibrationValid(float a0, float a1, float v0)
{
    if (!isfinite(a0) || !isfinite(a1) || !isfinite(v0) || fabsf(a0) > 100000 || fabsf(a1) > 10000 || v0 < 2 ||
        v0 > 60) {
        return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        const float voltage = v0 * (.8f + .2f * i);
        const float rc = a0 + a1 * voltage;
        const float collective = rcCalibrationCollective(rc);
        if (rc < 172 || rc > 1811 || !isfinite(collective) || collective < .1f || collective > .8f) {
            return false;
        }
    }
    return true;
}

void df3BetaflightReloadGains(void)
{
    const df3Config_t *p = df3Config();
    for (unsigned i = 0; i < 3; ++i) {
        controlConfig.kp[i] = p->kp[i] * .001f;
        controlConfig.kv[i] = p->kv[i] * .001f;
        controlConfig.ki[i] = p->ki[i] * .001f;
    }
    // A new tune starts with empty controller integrals, without restarting DF3.
    df3ControlReset(&controller);
}

void df3BetaflightReloadCalibration(void)
{
    // MSP calls this only while disarmed, after persistence/readback.
    controlConfig.hoverThrottle = df3Config()->hover * .0001f;
    df3ControlReset(&controller);
}

bool df3BetaflightAssistMappingReady(void)
{
    bool assigned = false;
    for (unsigned i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; ++i) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
        if (mac->auxChannelIndex != 2 || !IS_RANGE_USABLE(&mac->range) || mac->range.endStep <= 32 ||
            mac->range.startStep >= 48) {
            continue;
        }
        // A manual flash may retain the old AUX3 turtle-mode assignment.
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

static uint64_t df3TimeUs(void)
{
#ifdef SITL
    return micros64();
#else
    // Hardware exposes wrapping 32-bit micros. All callers are scheduler/PID
    // context, never the UART ISR. Extend continuously across its 71-minute wrap.
    static uint32_t previous;
    static uint64_t extended;
    const uint32_t current = micros();
    extended += (uint32_t)(current - previous);
    previous = current;
    return extended;
#endif
}

static void configureAssist(void)
{
    const df3Config_t *p = df3Config();
    df3BetaflightReloadGains();
    for (unsigned i = 0; i < 3; ++i) {
        controlConfig.integralLimit[i] = i < 2 ? DF3_CONTROL_XY_INTEGRAL_LIMIT_M_S : DF3_CONTROL_Z_INTEGRAL_LIMIT_M_S;
    }
    controlConfig.hoverThrottle = p->hover * .0001f;
    controlConfig.accelToThrottle = p->accelToThrottle * .0001f;
    controlConfig.maxTiltRad = p->maxTiltDeg * .01745329251994329577f;
#ifdef SITL
    const char *value = getenv("DFSIM_DF3_HOVER");
    if (value && *value) {
        char *end;
        const float hover = strtof(value, &end);
        if (*end || !isfinite(hover) || hover < .1f || hover > .8f) {
            fprintf(stderr, "Invalid DFSIM_DF3_HOVER collective calibration\n");
            exit(2);
        }
        controlConfig.hoverThrottle = hover;
    }
#endif
    // Default AUX3 high binding in the new feature only. Preserve every existing
    // slot and stored ID; an explicit configurator binding takes precedence.
    if (!isModeActivationConditionPresent(BOXFLIGHTASSIST)) {
        const modeActivationCondition_t empty = {0};
        for (unsigned i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; ++i) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            if (!memcmp(mac, &empty, sizeof(empty))) {
                *mac = (modeActivationCondition_t){.modeId = BOXFLIGHTASSIST,
                                                   .auxChannelIndex = 2,
                                                   .range = {.startStep = 32, .endStep = 48},
                                                   .modeLogic = MODELOGIC_OR};
                analyzeModeActivationConditions();
                break;
            }
        }
    }
}
#ifndef SITL
/* Device time has an unknown epoch. Minimum observed clock offset removes
 * positive polling jitter, but not undocumented internal sensor latency.
 * Hardware calibration must establish that latency before accuracy claims. */
static int64_t minimumClockOffset;
static bool clockAligned;
static uint64_t extendedSourceMs;
#endif

void df3BetaflightInit(void)
{
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
    memset(&blackboxFusion, 0, sizeof(blackboxFusion));
    memset(blackboxValues, 0, sizeof(blackboxValues));
    memset(blackboxForce, 0, sizeof(blackboxForce));
    blackboxControlUs = blackboxRangeReceiptUs = 0;
    blackboxSequence = blackboxRangeMm = 0;
    blackboxFlowClearance = 0;
    blackboxFlowQuality = 0;
#endif
    df3EstimatorReset(&estimator, 40000);
#ifdef USE_DF3_BUDGETED_WORKER
#ifdef SITL
    df3BudgetInit(&fusionWorker, 1000);
#else
    df3BudgetInit(&fusionWorker, clockMicrosToCycles(1));
#endif
#endif
#ifdef SITL
    const char *quantum = getenv("DFSIM_DF3_FLOW_QUANTIZATION_CMPS");
    flowQuantizationCMPS = 10;
    if (quantum && *quantum) {
        char *end;
        const float value = strtof(quantum, &end);
        if (*end || !isfinite(value) || value < 0 || value > 32767 || floorf(value) != value) {
            fprintf(stderr, "Invalid DFSIM_DF3_FLOW_QUANTIZATION_CMPS metadata\n");
            exit(2);
        }
        flowQuantizationCMPS = value;
    }
    fprintf(stderr, "DF3 flow quantization: %.0f cm/s at 1m\n", (double)flowQuantizationCMPS);
#endif
#ifdef USE_DF3_RESUMABLE
    df3FusionReset(&fusionJob);
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
    fusionJob.trace = &blackboxFusion;
#endif
#endif
    df3GyroHistoryReset(&gyroHistory);
#ifdef USE_DF3_MULTIRATE
    df3ImuReducerReset(&imuReducer);
    fusionJob.history = &gyroHistory;
#endif
    df3ReferenceReset(&referenceReceiver);
    df3ControlReset(&controller);
    memset(&controlOutput, 0, sizeof(controlOutput));
    controlUs = 0;
    configureAssist();
    stateEpoch = stateSequence = 0;
    memset(&referenceMailbox, 0, sizeof(referenceMailbox));
    referenceGeneration = 0;
    memset(&estimate, 0, sizeof(estimate));
    memset(&mtf, 0, sizeof(mtf));
    lastImuUs = lastAttitudeUs = lastRangeUs = lastGyroHistoryUs = 0;
    rangeGeneration = flowGeneration = flowAccepted = flowRejected = 0;
    memset(&lastFlow, 0, sizeof(lastFlow));
    wasArmed = activationQueued = false;
    booted = true;
#ifdef SITL
#ifdef USE_DF3_SCHED_BENCH
    if (!scheduleLog) {
        const char *path = getenv("DFSIM_DF3_SCHED_LOG");
        if (path && *path) {
            scheduleLog = fopen(path, "wx");
            if (!scheduleLog) {
                fprintf(stderr, "DF3 schedule log could not be created\n");
                exit(2);
            }
            setvbuf(scheduleLog, NULL, _IOFBF, 1024 * 1024);
            fprintf(scheduleLog, "time_us,worker,cpu_ns,phase,queue,job_age_us,covariance_age_us,busy,transactions,"
                                 "gyro_calls,pid_calls,gyro_max_gap_us,pid_max_gap_us");
#ifdef USE_DF3_BUDGETED_WORKER
            fprintf(scheduleLog, ",worker_phases,worker_budget_ns,worker_elapsed_ns,worker_yields,worker_overruns");
#endif
            fputc('\n', scheduleLog);
        }
    }
#endif
    if (!logFile) {
        const char *path = getenv("DFSIM_DF3_LOG");
        if (path && *path) {
            logFile = fopen(path, "wx");
            if (!logFile) {
                fprintf(stderr, "DF3 log could not be created\n");
                exit(2);
            }
            fprintf(logFile, "time_us,fusion_us,valid,vertical_valid");
            for (unsigned i = 0; i < 19; ++i) {
                fprintf(logFile, ",x%u", i);
            }
            fprintf(logFile, ",terrain_down,queued,predictions,updates,rejected,stale,"
                             "overflow,failed,flow_accepted,flow_rejected,flow_start_us,flow_"
                             "end_us,flow_p,flow_q,flow_forward,flow_right,reference_valid,reference_accepted,"
                             "reference_rejected,reference_epoch,reference_sequence,reference_source_ms,reference_"
                             "received_us,reference_x,reference_y,reference_z,assist_selected,assist_mode,assist_"
                             "authority,assist_roll,assist_pitch,assist_yaw_rate,assist_throttle");
#ifdef USE_DF3_MULTIRATE
            fprintf(logFile, ",raw_imu_us,raw_attitude_us,reduced_windows,reducer_gaps,reducer_invalid,epochs,total_"
                             "pending,history_faults,burst_faults,stationary_constraints,stationary");
#endif
            fprintf(logFile, ",range_m,hover_throttle,assist_accel_z,integral_z");
            fputc('\n', logFile);
        }
    }
#else
    clockAligned = false;
    extendedSourceMs = 0;
#endif
}

#if defined(USE_DF3_BUDGETED_WORKER) && defined(USE_DF3_PROFILE)
static uint32_t serviceAge(uint64_t now, uint64_t stamp)
{
    if (!stamp || now < stamp || now - stamp > UINT32_MAX) {
        return UINT32_MAX;
    }
    return now - stamp;
}

static void serviceCheckpoint(unsigned incomingKind)
{
    const unsigned pending = fusionJob.busy ? fusionJob.count - fusionJob.index : 0;
    const unsigned queue = estimator.count + pending;
    if (!df3ProfileServiceCheckpointNeeded(queue, estimator.failed)) {
        return;
    }
    const uint64_t now = df3TimeUs();
    const df3Profile_t *p = df3ProfileGet();
    uint64_t oldest = estimator.count ? estimator.events[0].us : 0;
    if (pending && (!oldest || fusionJob.batch[fusionJob.index].us < oldest)) {
        oldest = fusionJob.batch[fusionJob.index].us;
    }
    const df3ServiceSnapshot_t snapshot = {
        .elapsedUs = (uint32_t)now - p->startedUs,
        .jobAgeUs = serviceAge(now, fusionJob.startUs),
        .oldestAgeUs = serviceAge(now, oldest),
        .fusionAgeUs = serviceAge(now, estimator.fusionUs),
        .committedAgeUs = serviceAge(now, fusionJob.published ? fusionJob.committed.covarianceTimeUs : 0),
        .predictions = estimator.predictions,
        .updates = estimator.updates,
        .workerTicks =
            p->rows[DF3_PROF_WORKER].totalCycles > UINT32_MAX ? UINT32_MAX : p->rows[DF3_PROF_WORKER].totalCycles,
        .phases = p->budget.phases,
        .windows = p->service.windows,
        .shortWindows = p->service.shortWindows,
        .nextTicks = df3BudgetNextTicks(&fusionWorker, &fusionJob),
        .requiredUs = p->service.requiredUs,
        .availableUs = p->service.availableUs,
        .queue = queue,
        .inbox = estimator.count,
        .pending = pending,
        .phaseKey = df3FusionPhaseKey(&fusionJob),
        .corePhase = fusionJob.coreJob.phase,
        .coreRow = fusionJob.coreJob.row,
        .eventKind = pending ? fusionJob.batch[fusionJob.index].kind : DF3_SERVICE_NO_EVENT,
        .incomingKind = incomingKind,
        .coreStatus = fusionJob.coreJob.status,
        .failed = estimator.failed,
        .overflow = estimator.overflow,
        .busy = fusionJob.busy};
    df3ProfileFault_t fault = {0};
    if (estimator.failed) {
        if (estimator.overflow) {
            fault.reasons |= DF3_PROFILE_FAULT_QUEUE;
        }
#ifdef USE_DF3_MULTIRATE
        fault.reducerGaps = imuReducer.gaps;
        fault.reducerInvalid = imuReducer.invalid;
        fault.historyFaults = fusionJob.historyFaults;
        fault.burstFaults = fusionJob.burstFaults;
        if (fault.reducerGaps) {
            fault.reasons |= DF3_PROFILE_FAULT_IMU_GAP;
        }
        if (fault.reducerInvalid) {
            fault.reasons |= DF3_PROFILE_FAULT_IMU_INVALID;
        }
        if (fault.historyFaults) {
            fault.reasons |= DF3_PROFILE_FAULT_GYRO_HISTORY;
        }
        if (fault.burstFaults) {
            fault.reasons |= DF3_PROFILE_FAULT_EPOCH_BURST;
        }
#endif
        if (fusionJob.coreJob.status < DF3_INVALID_ARGUMENT) {
            fault.reasons |= DF3_PROFILE_FAULT_NUMERICAL;
        }
        if (!fault.reasons) {
            fault.reasons = DF3_PROFILE_FAULT_OTHER;
        }
    }
    df3ProfileServiceCheckpointDetailed(&snapshot, &fault);
}
#endif

static bool enqueue(df3EventKind_e kind, uint64_t us, const float *data, unsigned n)
{
    if (!estimator.initialized || estimator.failed) {
        return false;
    }
    df3Event_t event = {.us = us, .kind = kind};
    if (n) {
        memcpy(event.data, data, n * sizeof(float));
    }
#ifdef USE_DF3_RESUMABLE
    const bool accepted = df3FusionEnqueue(&fusionJob, &estimator, &event);
#if defined(USE_DF3_BUDGETED_WORKER) && defined(USE_DF3_PROFILE)
    df3ProfileServiceReceipt(kind, accepted);
    serviceCheckpoint(kind); // First queue failure, before cleanup discards busy.
#endif
    return accepted;
#else
    return df3EstimatorEnqueue(&estimator, &event);
#endif
}

void df3BetaflightAccelerometer(void)
{
    DF3_PROFILE_SCOPE(DF3_PROF_IMU_ADAPTER);
    if (!booted) {
        return;
    }
    const uint64_t now = df3TimeUs();
    const float rad = .01745329251994329577f;
    const float nativeGyro[3] = {gyroGetFilteredDownsampled(X), gyroGetFilteredDownsampled(Y),
                                 gyroGetFilteredDownsampled(Z)};
    float gyroFrd[3];
    df3NativeVectorToFrd(nativeGyro, rad, gyroFrd);
    if (now > lastGyroHistoryUs) {
        (void)df3GyroHistoryPush(&gyroHistory, now, gyroFrd);
        lastGyroHistoryUs = now;
    }
#ifndef USE_DF3_MULTIRATE
    if (lastImuUs && now - lastImuUs < 2000) {
        return;
    }
#endif
    const float g = 9.80665f * acc.dev.acc_1G_rec;
    const float nativeAccel[3] = {acc.accADC[X], acc.accADC[Y], acc.accADC[Z]};
    float data[6] = {gyroFrd[0], gyroFrd[1], gyroFrd[2]};
    df3NativeVectorToFrd(nativeAccel, g, data + 3);
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
    memcpy(blackboxForce, data + 3, sizeof(blackboxForce));
#endif
    memcpy(lastGyro, gyroFrd, sizeof(lastGyro));
    lastImuUs = now;
#ifdef USE_DF3_MULTIRATE
    const bool rangeFresh = lastRangeUs && now >= lastRangeUs && now - lastRangeUs <= 100000;
    const bool flowFresh = lastFlow.endUs && now >= lastFlow.endUs && now - lastFlow.endUs <= 100000;
    const bool externalStill = rangeFresh && flowFresh && mtf.quality && mtf.flowStatus &&
                               fabsf(lastFlow.correctedBodyVelocity[0]) < .08f &&
                               fabsf(lastFlow.correctedBodyVelocity[1]) < .08f;
    df3Event_t window;
    const int sealed =
        df3ImuReducerPush(&imuReducer, now, gyroFrd, data + 3, !ARMING_FLAG(ARMED) && !estimator.dynamicsActive,
                          externalStill, lastRange, &window);
    if (sealed < 0 && estimator.initialized) {
        estimator.failed = true;
#if defined(USE_DF3_BUDGETED_WORKER) && defined(USE_DF3_PROFILE)
        serviceCheckpoint(DF3_EVENT_IMU);
#endif
    }
    if (sealed > 0 && estimator.initialized && window.us >= estimator.bootstrapUs) {
        (void)enqueue(DF3_EVENT_IMU, window.us, window.data, 6);
    }
#else
    enqueue(DF3_EVENT_IMU, now, data, 6);
#endif
}

void df3BetaflightAttitude(void)
{
    DF3_PROFILE_SCOPE(DF3_PROF_ATT_ADAPTER);
    if (!booted) {
        return;
    }
    quaternion_t q;
    getQuaternion(&q);
    /* Equivalent to SDK Euler(roll,-BFpitch,BFyaw), avoiding wire quantization.
   */
    const float nativeQ[4] = {q.w, q.x, q.y, q.z};
    df3NativeQuaternionToFrd(nativeQ, lastQ);
    lastAttitudeUs = df3TimeUs();
#ifdef USE_DF3_MULTIRATE
    if (!df3AttitudeAdmit(&imuReducer, lastAttitudeUs)) {
        return;
    }
#endif
    enqueue(DF3_EVENT_ATTITUDE, lastAttitudeUs, lastQ, 4);
}

static uint32_t little32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void df3BetaflightMtfFrame(const uint8_t payload[20], uint8_t sequence)
{
    DF3_PROFILE_SCOPE(DF3_PROF_MTF_PARSE);
    if (!booted) {
        return;
    }
    const uint32_t sourceMs = little32(payload);
    const uint32_t deltaMs = sourceMs - mtf.previousSourceMs;
    const uint8_t deltaSeq = (uint8_t)(sequence - mtf.sequence);
    if (mtf.hasPrevious && deltaMs == 0 && deltaSeq == 0) {
        return;
    }
    mtf.timingValid = mtf.hasPrevious && deltaMs > 0 && deltaMs <= 250 && deltaSeq > 0 && deltaSeq <= 12;
    mtf.intervalUs = mtf.timingValid ? deltaMs * 1000 / deltaSeq : 0;
    if (mtf.intervalUs < 10000 || mtf.intervalUs > 50000) {
        mtf.timingValid = false;
    }
    mtf.receivedUs = df3TimeUs();
#ifdef SITL
    /* Clock provenance only. No aircraft state/velocity enters the estimator. */
    mtf.endUs = df3SitlSensorTimeUs(sourceMs);
#else
    if (!mtf.hasPrevious || deltaMs > 1000) {
        extendedSourceMs = sourceMs;
        clockAligned = false;
        mtf.timingValid = false;
    } else {
        extendedSourceMs += deltaMs;
    }
    const int64_t observed = (int64_t)mtf.receivedUs - (int64_t)(extendedSourceMs * 1000);
    if (!clockAligned || observed < minimumClockOffset) {
        minimumClockOffset = observed;
        clockAligned = true;
    }
    mtf.endUs = (uint64_t)((int64_t)(extendedSourceMs * 1000) + minimumClockOffset);
#endif
    mtf.previousSourceMs = sourceMs;
    mtf.sequence = sequence;
    mtf.hasPrevious = true;
    mtf.rangeMm = little32(payload + 4);
    mtf.rangeStatus = payload[10];
    mtf.flowStatus = payload[17];
    mtf.quality = payload[16];
    mtf.strength = payload[8];
    ++mtf.generation;
}

static bool mtfClearance(float *clearance)
{
    // rangeMm comes directly from the MTF packet, before the legacy rangefinder's
    // median filter and tilt correction. Apply tilt once to this raw distance;
    // the already-corrected altitude getter below is only a validity check.
    *clearance = .001f * (float)mtf.rangeMm * getCosTiltAngle();
    const uint64_t now = df3TimeUs();
    // Share the raw report and health checks; the legacy altitude includes a
    // median filter and integer-cm rounding, unsuitable for scaling this flow.
    return mtf.rangeStatus && isfinite(*clearance) && rangefinderGetLatestAltitudeMeters() > 0 &&
           *clearance > 0 && *clearance < 6 && mtf.endUs && now >= mtf.endUs && now - mtf.endUs <= 100000;
}

void df3BetaflightRange(void)
{
    DF3_PROFILE_SCOPE(DF3_PROF_RANGE_ADAPTER);
    if (!booted || !mtf.generation || mtf.generation == rangeGeneration) {
        return;
    }
    rangeGeneration = mtf.generation;
    float clearance;
    if (!mtfClearance(&clearance)) {
        return;
    }
    lastRange = -clearance;
    lastStrength = (float)mtf.strength;
    lastRangeUs = mtf.endUs;
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
    blackboxRangeMm = mtf.rangeMm;
    blackboxRangeReceiptUs = mtf.receivedUs;
#endif
    const float data[2] = {lastRange, lastStrength};
    /* DF3 owns range weighting. Fuse the unfiltered millimetre sample at
     * acquisition time; the legacy median remains available to other users. */
    enqueue(DF3_EVENT_RANGE, lastRangeUs, data, 2);
}

void df3BetaflightFlow(void)
{
    DF3_PROFILE_SCOPE(DF3_PROF_FLOW_ADAPTER);
    if (!booted || !mtf.generation || mtf.generation == flowGeneration) {
        return;
    }
    flowGeneration = mtf.generation;
    float clearance;
    const float normalized[2] = {.01f * (float)opticalflowGetLatestVelX(), -.01f * (float)opticalflowGetLatestVelY()};
    // Use the same fixed geometry for zero-bias compensation and fusion.
    const float scale = .001f * df3FlowConfig()->rotationScale;
    float offset[3];
    for (unsigned i = 0; i < 3; ++i) {
        offset[i] = .001f * df3FlowConfig()->sensorOffset[i];
    }
    df3FlowResult_t flow;
    if (!mtf.timingValid || !mtf.flowStatus || !mtf.quality || !mtfClearance(&clearance) || mtf.endUs < mtf.intervalUs ||
        !df3FlowCompensate(&gyroHistory, mtf.endUs - mtf.intervalUs, mtf.endUs, normalized, clearance, scale,
                           offset, &flow)) {
        ++flowRejected;
        return;
    }
    const float data[7] = {flow.correctedBodyVelocity[0],
                           flow.correctedBodyVelocity[1],
                           flow.averageGyro[0],
                           flow.averageGyro[1],
                           (float)mtf.quality,
                           scale * clearance,
                           (float)mtf.strength};
    if (enqueue(DF3_EVENT_FLOW, flow.midpointUs, data, 7)) {
        lastFlow = flow;
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
        blackboxFlowClearance = clearance;
        blackboxFlowQuality = mtf.quality;
#endif
        ++flowAccepted;
    } else {
        ++flowRejected;
    }
}

void df3BetaflightReferenceFrame(const uint8_t payload[DF3_REFERENCE_BYTES], uint32_t receivedUs)
{
    if (!booted) {
        return;
    }
    // UART ISR only transfers a bounded payload; decode/freshness run in the task.
    memcpy(referenceMailbox.payload, payload, DF3_REFERENCE_BYTES);
    referenceMailbox.receivedUs = receivedUs;
    ++referenceMailbox.generation;
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
        if (referenceGeneration != referenceMailbox.generation) {
            memcpy(payload, referenceMailbox.payload, sizeof(payload));
            stamp = referenceMailbox.receivedUs;
            referenceGeneration = referenceMailbox.generation;
            fresh = true;
        }
    }
    if (fresh) {
        const uint32_t age = (uint32_t)now - stamp;
        if (age <= 250000 && age <= now) {
            (void)df3ReferenceAccept(&referenceReceiver, payload, now - age, armed);
        }
    }
}

bool df3BetaflightReference(df3Reference_t *out)
{
    return df3ReferenceSample(&referenceReceiver, df3TimeUs(), out);
}

bool df3BetaflightAssistSelected(void)
{
    return booted && IS_RC_MODE_ACTIVE(BOXFLIGHTASSIST) && !failsafeIsActive() && !FLIGHT_MODE(GPS_RESCUE_MODE) &&
           !FLIGHT_MODE(HEADFREE_MODE) && !isFlipOverAfterCrashActive() && !isLaunchControlActive();
}

bool df3BetaflightAssistActive(void)
{
    return controlOutput.authority && ARMING_FLAG(ARMED) && rcData[THROTTLE] > rxConfig()->mincheck &&
           FLIGHT_MODE(ANGLE_MODE) && df3BetaflightAssistSelected();
}

const df3ControlOutput_t *df3BetaflightControl(void)
{
    // A stalled outer task must not silently restore pilot throttle while AUX3
    // still selects assist. Manual deselection/disarm retain their normal priority.
    static const df3ControlOutput_t stale = {.mode = DF3_CONTROL_FAULT, .authority = true};
    const uint64_t now = df3TimeUs();
    return now >= controlUs && now - controlUs <= 20000 ? &controlOutput : &stale;
}

#if defined(SITL) && defined(USE_DF3_SCHED_BENCH)
static void benchLog(bool worker, uint64_t started, unsigned phase)
{
    if (!scheduleLog) {
        return;
    }
    const uint64_t ns = benchCpuNs() - started, now = df3TimeUs();
    unsigned queue = estimator.count, busy = 0, transactions = 0;
    uint64_t age = 0;
#ifdef USE_DF3_RESUMABLE
    busy = fusionJob.busy;
    transactions = fusionJob.transactions;
    if (busy) {
        queue += fusionJob.count - fusionJob.index;
        age = now - fusionJob.startUs;
    }
#endif
    fprintf(scheduleLog, "%llu,%u,%llu,%u,%u,%llu,%llu,%u,%u,%llu,%llu,%u,%u", (unsigned long long)now, worker,
            (unsigned long long)ns, phase, queue, (unsigned long long)age,
            (unsigned long long)(now - estimate.covarianceTimeUs), busy, transactions,
            (unsigned long long)benchGyroCalls, (unsigned long long)benchPidCalls, benchGyroGap, benchPidGap);
#ifdef USE_DF3_BUDGETED_WORKER
    fprintf(scheduleLog, ",%u,%llu,%llu,%llu,%llu", worker ? fusionWorker.lastPhases : 0,
            (unsigned long long)(worker ? fusionWorker.lastBudgetTicks : 0),
            (unsigned long long)(worker ? fusionWorker.lastElapsedTicks : 0), (unsigned long long)fusionWorker.yields,
            (unsigned long long)fusionWorker.overruns);
#endif
    fputc('\n', scheduleLog);
}
#endif

#ifdef USE_DF3_RESUMABLE
bool df3BetaflightFusionReady(void)
{
#ifdef USE_DF3_MULTIRATE
    return fusionJob.busy || df3FusionDue(&fusionJob, &estimator, df3TimeUs());
#else
    return fusionJob.busy;
#endif
}
#ifdef USE_DF3_BUDGETED_WORKER
unsigned df3BetaflightFusionMinTimeUs(void)
{
    return df3BudgetMinimumUs(&fusionWorker, &fusionJob, &estimator);
}
unsigned df3BetaflightFusionBudgetUs(void)
{
    return df3BudgetCeilingUs(&fusionWorker, &fusionJob, &estimator);
}
#endif
void df3BetaflightFusionTick(void)
{
    DF3_PROFILE_SCOPE(DF3_PROF_WORKER);
#ifdef USE_DF3_MULTIRATE
    fusionJob.lastNowUs = df3TimeUs();
#endif
#ifdef USE_DF3_SCHED_BENCH
    const uint64_t started = scheduleLog ? benchCpuNs() : 0;
    const unsigned phase = fusionJob.phase == 2 ? 100 + fusionJob.coreJob.phase : fusionJob.phase;
#endif
#ifdef USE_DF3_BUDGETED_WORKER
    const unsigned available = schedulerTaskTimeAvailableUs();
    // Keep entry/exit bookkeeping outside the math budget. The scheduler retains
    // its own guard. Virtual deadlines and host CPU costs remain distinct clocks.
    const unsigned budget = df3BudgetOfferUs(&fusionWorker, &fusionJob, &estimator, available);
    df3BudgetRun(&fusionWorker, &fusionJob, &estimator, df3BudgetMathTicks(&fusionWorker, budget), fusionClock, NULL);
#ifdef USE_DF3_PROFILE
    df3ProfileBudget(fusionWorker.lastPhases, fusionJob.busy, fusionWorker.lastBudgetTicks,
                     fusionWorker.lastElapsedTicks, df3BudgetNextTicks(&fusionWorker, &fusionJob),
                     df3FusionPhaseKey(&fusionJob));
    serviceCheckpoint(DF3_SERVICE_NO_EVENT); // Includes numerical worker failures.
#endif
#else
    /* A phase may be only a state transition. Four bounded phases avoid
   * exhausting native scheduler opportunities on those transitions alone.
   * This is a work bound, not a claim of a hardware microsecond deadline. */
    for (unsigned i = 0; i < 4 && fusionJob.busy; ++i) {
        df3FusionResume(&fusionJob, &estimator);
    }
#endif
#ifdef USE_DF3_PROFILE
    DF3_PROFILE_GAUGE(DF3_PROF_QUEUE_OUT, estimator.count + (fusionJob.busy ? fusionJob.count - fusionJob.index : 0));
    const uint64_t now = df3TimeUs();
    if (now >= fusionJob.startUs) {
        DF3_PROFILE_GAUGE(DF3_PROF_TRANSACTION_AGE_US, now - fusionJob.startUs);
    }
#endif
#ifdef USE_DF3_SCHED_BENCH
    benchLog(true, started, phase);
#endif
}
#endif

void df3BetaflightTick(void)
{
#if defined(SITL) && defined(USE_DF3_SCHED_BENCH)
    const uint64_t started = scheduleLog ? benchCpuNs() : 0;
#endif
#ifdef USE_DF3_PROFILE
    df3ProfilePoll();
#endif
    DF3_PROFILE_SCOPE(DF3_PROF_TASK);
    if (!booted) {
        df3BetaflightInit();
    }
    const uint64_t now = df3TimeUs();
    const bool armed = ARMING_FLAG(ARMED);
    consumeReference(now, armed);
    if (wasArmed && !armed) {
#ifdef USE_DF3_RESUMABLE
        df3FusionReset(&fusionJob);
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
        memset(&blackboxFusion, 0, sizeof(blackboxFusion));
        fusionJob.trace = &blackboxFusion;
#endif
#endif
        df3EstimatorReset(&estimator, 40000);
#ifdef USE_DF3_MULTIRATE
        df3ImuReducerReset(&imuReducer);
        fusionJob.history = &gyroHistory;
#endif
        activationQueued = false;
    }
    wasArmed = armed;
    if (!estimator.initialized && !armed && lastImuUs && lastAttitudeUs && lastRangeUs && now - lastImuUs <= 200000 &&
        now - lastAttitudeUs <= 250000 && now - lastRangeUs <= 300000) {
#ifdef USE_DF3_PROFILE_AUTOSTART
        df3ProfileAutoStart(); // first usable sensor set, once per boot; retained for CLI readback
#endif
        if (df3EstimatorInitialize(&estimator, now, lastRange, lastQ, lastGyro, lastStrength)) {
            estimator.flowHeightPerGain = 1.f / (.001f * df3FlowConfig()->rotationScale);
            estimator.flowQuantumPerGain = .01f * flowQuantizationCMPS * estimator.flowHeightPerGain;
            for (unsigned i = 0; i < 3; ++i) {
                estimator.flowSensorOffset[i] = .001f * df3FlowConfig()->sensorOffset[i];
            }
#ifdef USE_DF3_MULTIRATE
            df3ImuReducerReset(&imuReducer);
            fusionJob.history = &gyroHistory;
#endif
            if (++stateEpoch == 0) {
                ++stateEpoch;
            }
        }
    }
    if (armed && !activationQueued && rcData[THROTTLE] > rxConfig()->mincheck) {
        activationQueued = enqueue(DF3_EVENT_ACTIVATE, now, 0, 0);
    }
#ifdef USE_DF3_RESUMABLE
    // Queue overflow can be latched by a sensor adapter while the job is paused.
    // Finish its bounded fault cleanup here even if no math-sized slot is offered.
    if (fusionJob.busy && estimator.failed) {
#if defined(USE_DF3_BUDGETED_WORKER) && defined(USE_DF3_PROFILE)
        serviceCheckpoint(DF3_SERVICE_NO_EVENT);
#endif
        df3FusionResume(&fusionJob, &estimator);
    }
    DF3_PROFILE_GAUGE(DF3_PROF_QUEUE_IN, estimator.count + (fusionJob.busy ? fusionJob.count - fusionJob.index : 0));
#if defined(USE_DF3_MULTIRATE) && defined(USE_DF3_BUDGETED_WORKER)
    // Epoch starts are measured worker phases, not gated on foreground service.
    fusionJob.lastNowUs = now;
#else
    (void)df3FusionStart(&fusionJob, &estimator, now);
#if defined(USE_DF3_BUDGETED_WORKER) && defined(USE_DF3_PROFILE)
    serviceCheckpoint(DF3_SERVICE_NO_EVENT);
#endif
#endif
    (void)df3FusionOutput(&fusionJob, &estimator, now, &estimate);
    if (now >= estimate.covarianceTimeUs) {
        DF3_PROFILE_GAUGE(DF3_PROF_FUSION_AGE_US, now - estimate.covarianceTimeUs);
    }
#else
    (void)df3EstimatorTick(&estimator, now, &estimate);
#endif
#ifdef USE_DF3_PROFILE
    if (df3ProfileGet()->active) {
        const df3ProfileHealth_t health = {.initialized = estimator.initialized,
                                           .failed = estimator.failed,
                                           .dynamicsActive = estimator.dynamicsActive,
                                           .valid = estimate.valid,
                                           .verticalValid = estimate.verticalReferenceValid,
                                           .predictions = estimator.predictions,
                                           .updates = estimator.updates,
                                           .rejected = estimator.rejected,
                                           .stale = estimator.stale,
                                           .overflow = estimator.overflow,
                                           .mtfFrames = mtf.generation,
                                           .flowAccepted = flowAccepted,
                                           .flowRejected = flowRejected,
                                           .rangeStatus = mtf.rangeStatus,
                                           .flowStatus = mtf.flowStatus,
                                           .flowQuality = mtf.quality};
        df3ProfileHealth(&health);
    }
#endif
    if (!controlUs || now - controlUs >= DF3_CONTROL_PERIOD_US) {
        DF3_PROFILE_SCOPE(DF3_PROF_CONTROL);
        df3Reference_t reference;
        const bool haveReference = df3ReferenceSample(&referenceReceiver, now, &reference);
        const bool imuFresh = lastImuUs && now >= lastImuUs && now - lastImuUs <= 20000;
        const bool rangeFresh = lastRangeUs && now >= lastRangeUs && now - lastRangeUs <= 200000;
        // A rejected report does not erase a recent accepted measurement.
        // Sustained loss still expires at the existing 150 ms bound.
        const bool flowFresh = lastFlow.endUs &&
                               now >= lastFlow.endUs && now - lastFlow.endUs <= 150000;
        const df3CalibrationConfig_t *cal = df3CalibrationConfig();
        if (cal->enabled) {
            if (df3BetaflightCalibrationValid(cal->a0, cal->a1, cal->v0)) {
                float voltage = getBatteryVoltage() * .01f; // BF battery API is centivolts
                if (voltage < 2) {
                    voltage = cal->v0; // no voltage source: reference-voltage trim
                }
                voltage = fmaxf(.8f * cal->v0, fminf(1.2f * cal->v0, voltage));
                controlConfig.hoverThrottle = df3BetaflightCalibrationHover(cal->a0, cal->a1, voltage);
            } else {
                controlConfig.hoverThrottle = 0;
            }
        }
        df3ControlStep(&controller, &controlConfig, now, df3BetaflightAssistSelected(), armed,
                       rcData[THROTTLE] > rxConfig()->mincheck, failsafeIsActive(), imuFresh, rangeFresh, flowFresh,
                       &estimate, haveReference ? &reference : NULL, &controlOutput);
        const df3DiagnosticsInput_t diagnosticInput = {.armed = armed,
                                                       .selected = df3BetaflightAssistSelected(),
                                                       .permit = rcData[THROTTLE] > rxConfig()->mincheck,
                                                       .imuFresh = imuFresh,
                                                       .rangeFresh = rangeFresh,
                                                       .flowFresh = flowFresh,
                                                       .fusionFresh = estimate.covarianceTimeUs &&
                                                                      now >= estimate.covarianceTimeUs &&
                                                                      now - estimate.covarianceTimeUs <= 150000,
                                                       .estimatorValid = estimate.valid && !estimator.failed,
                                                       .referenceValid = haveReference && reference.active,
                                                       .controllerFault = controlOutput.mode == DF3_CONTROL_FAULT,
                                                       .nativeFailsafe = failsafeIsActive(),
                                                       .overflow = estimator.overflow,
                                                       .rejected = referenceReceiver.rejected};
        df3DiagnosticsUpdate(&diagnostics, micros(), &diagnosticInput);
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
        captureBlackbox(now, haveReference);
#endif
        controlUs = now;
    }
#if defined(SITL) && defined(USE_DF3_SCHED_BENCH)
    benchLog(false, started, 0);
#endif
#ifdef SITL
    if (logFile) {
        fprintf(logFile, "%llu,%llu,%u,%u", (unsigned long long)now, (unsigned long long)estimate.covarianceTimeUs,
                estimate.valid, estimate.verticalReferenceValid);
        for (unsigned i = 0; i < 19; ++i) {
            fprintf(logFile, ",%.9g", (double)estimate.x[i]);
        }
        fprintf(logFile, ",%.9g,%u,%u,%u,%u,%u,%u,%u,%u,%u,%llu,%llu,%.9g,%.9g,%.9g,%.9g", (double)estimate.terrainDown,
                estimator.count, estimator.predictions, estimator.updates, estimator.rejected, estimator.stale,
                estimator.overflow, estimator.failed, flowAccepted, flowRejected, (unsigned long long)lastFlow.startUs,
                (unsigned long long)lastFlow.endUs, (double)lastFlow.averageGyro[0], (double)lastFlow.averageGyro[1],
                (double)lastFlow.correctedBodyVelocity[0], (double)lastFlow.correctedBodyVelocity[1]);
        df3Reference_t sampled;
        const bool referenceValid = df3ReferenceSample(&referenceReceiver, now, &sampled);
        fprintf(logFile, ",%u,%u,%u,%u,%u,%u,%llu,%.9g,%.9g,%.9g", referenceValid, referenceReceiver.accepted,
                referenceReceiver.rejected, referenceReceiver.reference.epoch, referenceReceiver.reference.sequence,
                referenceReceiver.reference.sourceMs, (unsigned long long)referenceReceiver.receivedUs,
                (double)referenceReceiver.reference.position[0], (double)referenceReceiver.reference.position[1],
                (double)referenceReceiver.reference.position[2]);
        fprintf(logFile, ",%u,%u,%u,%.9g,%.9g,%.9g,%.9g", df3BetaflightAssistSelected(), controlOutput.mode,
                df3BetaflightAssistActive(), (double)controlOutput.angleDeg[0], (double)controlOutput.angleDeg[1],
                (double)controlOutput.yawRateDeg, (double)controlOutput.throttle);
#ifdef USE_DF3_MULTIRATE
        fprintf(logFile, ",%llu,%llu,%u,%u,%u,%u,%u,%u,%u,%u,%u", (unsigned long long)lastImuUs,
                (unsigned long long)lastAttitudeUs, imuReducer.windows, imuReducer.gaps, imuReducer.invalid,
                fusionJob.epochs, estimator.count + (fusionJob.busy ? fusionJob.count - fusionJob.index : 0),
                fusionJob.historyFaults, fusionJob.burstFaults, fusionJob.stationaryUpdates, estimator.stationary);
#endif
        fprintf(logFile, ",%.9g,%.9g,%.9g,%.9g", (double)-lastRange, (double)controlConfig.hoverThrottle,
                (double)controlOutput.acceleration[2], (double)controller.integral[2]);
        fputc('\n', logFile);
    }
#endif
}

const df3Estimate_t *df3BetaflightEstimate(void)
{
    return &estimate;
}
void df3BetaflightStatePayload(uint8_t payload[DF3_STATE_BYTES])
{
    df3StateEncode(&estimate, stateEpoch, stateSequence++, ARMING_FLAG(ARMED), df3BetaflightAssistActive(), payload);
}
void df3BetaflightDiagnosticsPayload(uint8_t payload[DF3_DIAGNOSTICS_BYTES])
{
    df3DiagnosticsEncode(&diagnostics, (uint32_t)(df3TimeUs() / 1000), payload);
}
void df3BetaflightTelemetryPoll(uint32_t now)
{
    df3DiagnosticsTelemetryPoll(&diagnostics, now);
}
void df3BetaflightTelemetryQueued(uint32_t now, bool replacing)
{
    df3DiagnosticsTelemetryQueued(&diagnostics, now, replacing);
}
void df3BetaflightUartSubmitted(uint32_t now, uint8_t type)
{
    df3DiagnosticsUartSubmitted(&diagnostics, now, type);
}
#endif

#if defined(USE_DF3) && defined(USE_DF3_PROFILE)
#include "drivers/system.h"
#ifdef SITL
#include <time.h>
// SITL's scheduler cycle counter uses virtual time and cannot measure CPU work.
uint32_t df3ProfileReadCycles(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint32_t)((uint64_t)t.tv_sec * 1000000000ULL + t.tv_nsec);
}
uint32_t df3ProfileCyclesPerUs(void)
{
    return 1000;
}
#else
uint32_t df3ProfileReadCycles(void)
{
    return getCycleCounter();
}
uint32_t df3ProfileCyclesPerUs(void)
{
    return clockMicrosToCycles(1);
}
#endif
uint32_t df3ProfileReadMicros(void)
{
    return micros();
}
#endif
