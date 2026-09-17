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
    uint32_t generation, intervalUs, previousSourceMs;
    uint8_t sequence;
    bool timingValid, hasPrevious;
    uint8_t rangeStatus, flowStatus, quality, strength;
} mtf;
static uint32_t rangeGeneration, flowGeneration, flowAccepted, flowRejected;
/* MTF wire units are normalized cm/s. A coarser hardware acquisition grid is
 * not established by telemetry alone; do not assume the sim's 10-count grid. */
static float flowQuantizationCMPS = 1;

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
    for (unsigned i = 0; i < 3; ++i) {
        controlConfig.kp[i] = p->kp[i] * .001f;
        controlConfig.kv[i] = p->kv[i] * .001f;
        controlConfig.ki[i] = p->ki[i] * .001f;
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
    flowQuantizationCMPS = 1;
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
    mtf.rangeStatus = payload[10];
    mtf.flowStatus = payload[17];
    mtf.quality = payload[16];
    mtf.strength = payload[8];
    ++mtf.generation;
}

void df3BetaflightRange(void)
{
    DF3_PROFILE_SCOPE(DF3_PROF_RANGE_ADAPTER);
    if (!booted || !mtf.generation || mtf.generation == rangeGeneration) {
        return;
    }
    rangeGeneration = mtf.generation;
    const float clearance = rangefinderGetLatestAltitudeMeters();
    if (!mtf.rangeStatus || !isfinite(clearance) || clearance <= 0 || df3TimeUs() - mtf.receivedUs > 100000) {
        return;
    }
    lastRange = -clearance;
    lastStrength = (float)mtf.strength;
    lastRangeUs = df3TimeUs();
    const float data[2] = {lastRange, lastStrength};
    /* Preserve median filtering and tilt correction at the MTF's mm precision.
   * Its timestamp is processing time, not a fictitious raw capture time. */
    enqueue(DF3_EVENT_RANGE, lastRangeUs, data, 2);
}

void df3BetaflightFlow(void)
{
    DF3_PROFILE_SCOPE(DF3_PROF_FLOW_ADAPTER);
    if (!booted || !mtf.generation || mtf.generation == flowGeneration) {
        return;
    }
    flowGeneration = mtf.generation;
    const int32_t cm = rangefinderGetLatestAltitude();
    const uint64_t now = df3TimeUs();
    const float normalized[2] = {.01f * (float)opticalflowGetLatestVelX(), -.01f * (float)opticalflowGetLatestVelY()};
    // Use the same fixed geometry for zero-bias compensation and fusion.
    const float scale = .001f * df3FlowConfig()->rotationScale;
    float offset[3];
    for (unsigned i = 0; i < 3; ++i) {
        offset[i] = .001f * df3FlowConfig()->sensorOffset[i];
    }
    if (!mtf.timingValid || !mtf.rangeStatus || !mtf.flowStatus || cm <= 0 || mtf.endUs < mtf.intervalUs ||
        now < mtf.endUs || now - mtf.endUs > 100000 ||
        !df3FlowCompensate(&gyroHistory, mtf.endUs - mtf.intervalUs, mtf.endUs, normalized, (float)cm * .01f, scale,
                           offset, &lastFlow)) {
        ++flowRejected;
        return;
    }
    const float data[6] = {lastFlow.correctedBodyVelocity[0],
                           lastFlow.correctedBodyVelocity[1],
                           lastFlow.averageGyro[0],
                           lastFlow.averageGyro[1],
                           (float)mtf.quality,
                           scale * (float)cm * .01f};
    if (enqueue(DF3_EVENT_FLOW, lastFlow.midpointUs, data, 6)) {
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
            estimator.flowQuantumPerGain = .01f * flowQuantizationCMPS / (.001f * df3FlowConfig()->rotationScale);
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
        const bool flowFresh = mtf.flowStatus && mtf.quality && mtf.rangeStatus && lastFlow.endUs &&
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
