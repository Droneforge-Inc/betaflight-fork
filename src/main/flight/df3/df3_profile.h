/* Optional bench instrumentation. No estimator state or scheduling decisions. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DF3_PROF_TASK,
    DF3_PROF_INITIALIZE,
    DF3_PROF_FUSION,
    DF3_PROF_PREDICT,
    DF3_PROF_ZERO_ACCEL,
    DF3_PROF_ACCEL,
    DF3_PROF_ZERO_VEL,
    DF3_PROF_ATTITUDE,
    DF3_PROF_POSITION,
    DF3_PROF_FLOW_UPDATE,
    DF3_PROF_EVENT_IMU,
    DF3_PROF_EVENT_ATTITUDE,
    DF3_PROF_EVENT_RANGE,
    DF3_PROF_EVENT_FLOW,
    DF3_PROF_EVENT_ACTIVATE,
    DF3_PROF_OUTPUT,
    DF3_PROF_ENQUEUE,
    DF3_PROF_IMU_ADAPTER,
    DF3_PROF_ATT_ADAPTER,
    DF3_PROF_RANGE_ADAPTER,
    DF3_PROF_FLOW_ADAPTER,
    DF3_PROF_FLOW_COMP,
    DF3_PROF_MTF_PARSE,
    DF3_PROF_CONTROL,
    DF3_PROF_PREDICT_MODEL,
    DF3_PROF_PREDICT_COV,
    DF3_PROF_JOSEPH,
    DF3_PROF_RESET_COV,
    DF3_PROF_PROBE,
    DF3_PROF_WORKER,
    DF3_PROF_CORE_SLICE,
    DF3_PROF_GAIN_SLICE,
    DF3_PROF_COUNT
} df3ProfileSection_e;
typedef enum {
    DF3_PROF_QUEUE_IN,
    DF3_PROF_QUEUE_OUT,
    DF3_PROF_EVENTS_TICK,
    DF3_PROF_OLDEST_AGE_US,
    DF3_PROF_FUSION_AGE_US,
    DF3_PROF_TRANSACTION_AGE_US,
    DF3_PROF_GYRO_GAP_US,
    DF3_PROF_PID_GAP_US,
    DF3_PROF_GAUGE_COUNT
} df3ProfileGauge_e;

#ifdef USE_DF3_PROFILE
enum { DF3_PROFILE_BINS = 17, DF3_PROFILE_WINDOW_US = 20000000 };
typedef struct {
    uint64_t totalCycles;
    uint32_t calls, minCycles, maxCycles, histogram[DF3_PROFILE_BINS];
} df3ProfileRow_t;
typedef struct {
    uint64_t sum;
    uint32_t samples, last, max;
} df3ProfileGauge_t;
typedef struct {
    uint32_t initialized, failed, dynamicsActive, valid, verticalValid;
    uint32_t predictions, updates, rejected, stale, overflow;
    uint32_t mtfFrames, flowAccepted, flowRejected, rangeStatus, flowStatus, flowQuality;
} df3ProfileHealth_t;
#ifdef USE_DF3_BUDGETED_WORKER
typedef struct {
    uint32_t calls, phases, yields, overruns, maxPhases;
    uint32_t maxElapsedTicks, maxOverrunTicks, maxNextTicks, maxNextKey, zeroPhaseCalls;
} df3ProfileBudget_t;
enum { DF3_SERVICE_BUCKETS = 38, DF3_SERVICE_WINDOWS = 6, DF3_SERVICE_SNAPSHOTS = 3, DF3_SERVICE_NO_EVENT = 255 };
typedef struct {
    uint32_t calls, totalTicks, maxTicks;
} df3ServiceCost_t;
typedef struct {
    uint32_t elapsedUs, jobAgeUs, oldestAgeUs, fusionAgeUs, committedAgeUs;
    uint32_t predictions, updates, workerTicks, phases, windows, shortWindows, nextTicks;
    uint16_t requiredUs, availableUs;
    uint8_t queue, inbox, pending, phaseKey, corePhase, coreRow;
    uint8_t eventKind, incomingKind, coreStatus, failed, overflow, busy;
} df3ServiceSnapshot_t;
enum {
    DF3_PROFILE_FAULT_QUEUE = 1U << 0,
    DF3_PROFILE_FAULT_IMU_GAP = 1U << 1,
    DF3_PROFILE_FAULT_IMU_INVALID = 1U << 2,
    DF3_PROFILE_FAULT_GYRO_HISTORY = 1U << 3,
    DF3_PROFILE_FAULT_EPOCH_BURST = 1U << 4,
    DF3_PROFILE_FAULT_NUMERICAL = 1U << 5,
    DF3_PROFILE_FAULT_OTHER = 1U << 6
};
typedef struct {
    uint32_t reasons, observedUs;
    uint32_t reducerGaps, reducerInvalid, historyFaults, burstFaults;
    bool timingActive;
} df3ProfileFault_t;
typedef struct {
    df3ServiceCost_t cost[DF3_SERVICE_BUCKETS];
    df3ServiceSnapshot_t snapshot[DF3_SERVICE_SNAPSHOTS];
    uint32_t arrived[5], accepted[5], windowBins[DF3_SERVICE_WINDOWS];
    uint32_t windows, shortWindows, otherSelected, bookkeepingTicks, saturated;
    uint16_t availableUs, requiredUs;
    uint8_t snapshotMask;
    df3ProfileFault_t fault;
} df3ProfileService_t;
_Static_assert(sizeof(df3ProfileService_t) <= 784, "Keep G4 diagnostic RAM bounded");
#endif
typedef struct {
    df3ProfileRow_t rows[DF3_PROF_COUNT];
    df3ProfileGauge_t gauges[DF3_PROF_GAUGE_COUNT];
    df3ProfileHealth_t first, last;
    uint64_t bookkeepingCycles;
    uint32_t startedUs, stoppedUs, cyclesPerUs, generation;
    bool active, hasHealth;
#ifdef USE_DF3_BUDGETED_WORKER
    df3ProfileBudget_t budget;
    df3ProfileService_t service;
#endif
} df3Profile_t;
typedef struct {
    uint32_t started, generation;
    df3ProfileSection_e section;
} df3ProfileScope_t;

/* All instrumentation runs in cooperative scheduler/CLI context, never ISR.
 * Counts include interrupts and nested scopes. Do not sum parent/child rows.
 * Unsigned subtraction handles one cycle-counter wrap within each operation.
 */
uint32_t df3ProfileReadCycles(void);
uint32_t df3ProfileReadMicros(void);
uint32_t df3ProfileCyclesPerUs(void);
void df3ProfileStart(void);
/* Explicit destructive diagnostic reset. Never resets the estimator. Start
 * alone preserves a captured first fault until reset or a hardware reboot. */
void df3ProfileRestart(void);
#ifdef USE_DF3_PROFILE_AUTOSTART
void df3ProfileAutoStart(void);
#endif
#ifdef USE_DF3_BUDGETED_WORKER
void df3ProfileBudget(unsigned phases, bool busy, uint32_t budgetTicks, uint32_t elapsedTicks, uint32_t nextTicks,
                      unsigned nextKey);
bool df3ProfileServiceActive(void);
unsigned df3ProfileServiceBucket(unsigned phase, unsigned corePhase, unsigned eventKind);
void df3ProfileServiceOperation(unsigned bucket, uint32_t elapsedTicks);
void df3ProfileServiceWindow(unsigned requiredUs, unsigned availableUs, bool selected);
void df3ProfileServiceReceipt(unsigned kind, bool accepted);
bool df3ProfileServiceCheckpointNeeded(unsigned queue, bool failed);
void df3ProfileServiceCheckpoint(const df3ServiceSnapshot_t *snapshot);
void df3ProfileServiceCheckpointDetailed(const df3ServiceSnapshot_t *snapshot, const df3ProfileFault_t *fault);
#endif
void df3ProfileStop(void);
void df3ProfilePoll(void);
df3ProfileScope_t df3ProfileBegin(df3ProfileSection_e section);
void df3ProfileEnd(df3ProfileScope_t *scope);
void df3ProfileGauge(df3ProfileGauge_e gauge, uint64_t value);
void df3ProfileHealth(const df3ProfileHealth_t *health);
const df3Profile_t *df3ProfileGet(void);
const char *df3ProfileName(df3ProfileSection_e section);
const char *df3ProfileGaugeName(df3ProfileGauge_e gauge);
uint32_t df3ProfileP95UpperUs(const df3ProfileRow_t *row);

/* GCC/Clang cleanup records all early returns without restructuring numerical
 * code. One scope per lexical block. Disabled builds contain no timing calls. */
#define DF3_PROFILE_NAME_IMPL(a, b) a##b
#define DF3_PROFILE_NAME(a, b) DF3_PROFILE_NAME_IMPL(a, b)
#define DF3_PROFILE_SCOPE(section)                                                                                     \
    df3ProfileScope_t DF3_PROFILE_NAME(df3ProfileScope, __COUNTER__) __attribute__((cleanup(df3ProfileEnd))) =         \
        df3ProfileBegin(section)
#define DF3_PROFILE_GAUGE(gauge, value) df3ProfileGauge(gauge, value)
#else
#define DF3_PROFILE_SCOPE(section)                                                                                     \
    do {                                                                                                               \
    } while (0)
#define DF3_PROFILE_GAUGE(gauge, value)                                                                                \
    do {                                                                                                               \
    } while (0)
#endif
