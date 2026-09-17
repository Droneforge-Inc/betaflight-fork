#include "df3_profile.h"
#ifdef USE_DF3_PROFILE
#include <limits.h>
#include <string.h>

static df3Profile_t profile;
static uint32_t generation;
static const char *const names[DF3_PROF_COUNT] = {"task",
                                                  "initialize",
                                                  "fusion",
                                                  "predict",
                                                  "correct_zero_accel",
                                                  "correct_accel",
                                                  "correct_zero_vel",
                                                  "correct_attitude",
                                                  "correct_position",
                                                  "correct_flow",
                                                  "event_imu",
                                                  "event_attitude",
                                                  "event_range",
                                                  "event_flow",
                                                  "event_activate",
                                                  "project_output",
                                                  "enqueue",
                                                  "adapter_imu",
                                                  "adapter_attitude",
                                                  "adapter_range",
                                                  "adapter_flow",
                                                  "flow_compensate",
                                                  "mtf_parse",
                                                  "controller",
                                                  "predict_model",
                                                  "predict_covariance",
                                                  "update_joseph",
                                                  "update_reset_cov",
                                                  "empty_probe",
                                                  "fusion_worker",
                                                  "core_slice",
                                                  "gain_slice"};
static const char *const gaugeNames[DF3_PROF_GAUGE_COUNT] = {
    "queue_in",      "queue_out",          "events_per_tick", "oldest_event_age_us",
    "fusion_age_us", "transaction_age_us", "gyro_gap_us",     "pid_gap_us"};

void df3ProfileStart(void)
{
#ifdef USE_DF3_BUDGETED_WORKER
    // A routine timing capture must not destroy the first failure evidence.
    if (profile.service.snapshotMask & 4) {
        return;
    }
#endif
    memset(&profile, 0, sizeof(profile));
    if (++generation == 0) {
        ++generation;
    }
    profile.generation = generation;
    profile.cyclesPerUs = df3ProfileCyclesPerUs();
    profile.startedUs = df3ProfileReadMicros();
    profile.active = profile.cyclesPerUs != 0;
    /* Measure an empty instrumented interval. Its bookkeeping is reported
     * separately; neither is subtracted from actual task measurements. */
    for (unsigned i = 0; i < 32; ++i) {
        DF3_PROFILE_SCOPE(DF3_PROF_PROBE);
    }
}

void df3ProfileRestart(void)
{
#ifdef USE_DF3_BUDGETED_WORKER
    profile.service.snapshotMask = 0;
#endif
    df3ProfileStart();
}

void df3ProfileStop(void)
{
    if (profile.active) {
        profile.stoppedUs = df3ProfileReadMicros();
        profile.active = false;
    }
}

#ifdef USE_DF3_PROFILE_AUTOSTART
void df3ProfileAutoStart(void)
{
    // Never restart/overwrite an existing capture, even after disarm or a
    // failed initialization. Explicit CLI start remains available afterwards.
    if (!generation) {
        df3ProfileStart();
    }
}
#endif

#ifdef USE_DF3_BUDGETED_WORKER
bool df3ProfileServiceActive(void)
{
    return profile.active && !(profile.service.snapshotMask & 4);
}

static void serviceAdd(uint32_t *value, uint32_t increment)
{
    if (UINT32_MAX - *value < increment) {
        *value = UINT32_MAX;
        profile.service.saturated = 1;
    } else {
        *value += increment;
    }
}

unsigned df3ProfileServiceBucket(unsigned phase, unsigned corePhase, unsigned eventKind)
{
#ifdef USE_DF3_MULTIRATE
    // Reuse three previously reserved buckets without expanding retained RAM.
    if (phase == 16) {
        return 0; // bounded current-state gyro projection
    }
    if (phase == 0) {
        return 2; // epoch admission/start, before the job is busy
    }
    if (phase == 12) {
        return 20; // window anchoring (core phase 8 unused)
    }
    if (phase == 13) {
        return 21; // window accumulation (core phase 9 unused)
    }
    if (phase == 15) {
        return 12; // flow likelihood (core phase 0 unused)
    }
    if (phase == 14) {
        return 31; // window observation (core phase 19 unused)
    }
#endif
    // Keep row groups together, but distinguish event preparation by sensor.
    if (phase == 2) {
        return corePhase <= 20 ? 12 + corePhase : 0;
    }
    if (phase == 3) {
        return eventKind < 5 ? 33 + eventKind : 0;
    }
    return phase <= 11 ? phase : 0;
}

void df3ProfileServiceOperation(unsigned bucket, uint32_t elapsedTicks)
{
    if (!df3ProfileServiceActive() || bucket >= DF3_SERVICE_BUCKETS) {
        return;
    }
    const uint32_t started = df3ProfileReadCycles();
    df3ServiceCost_t *row = &profile.service.cost[bucket];
    serviceAdd(&row->calls, 1);
    serviceAdd(&row->totalTicks, elapsedTicks);
    if (elapsedTicks > row->maxTicks) {
        row->maxTicks = elapsedTicks;
    }
    serviceAdd(&profile.service.bookkeepingTicks, df3ProfileReadCycles() - started);
}

void df3ProfileServiceWindow(unsigned requiredUs, unsigned availableUs, bool selected)
{
    if (!df3ProfileServiceActive()) {
        return;
    }
    const uint32_t started = df3ProfileReadCycles();
    df3ProfileService_t *s = &profile.service;
    serviceAdd(&s->windows, 1);
    serviceAdd(&s->shortWindows, availableUs < requiredUs);
    serviceAdd(&s->otherSelected, !selected);
    const unsigned bin = availableUs / 32;
    serviceAdd(&s->windowBins[bin < 5 ? bin : 5], 1);
    s->availableUs = availableUs > UINT16_MAX ? UINT16_MAX : availableUs;
    s->requiredUs = requiredUs > UINT16_MAX ? UINT16_MAX : requiredUs;
    serviceAdd(&s->bookkeepingTicks, df3ProfileReadCycles() - started);
}

void df3ProfileServiceReceipt(unsigned kind, bool accepted)
{
    if (!df3ProfileServiceActive() || kind >= 5) {
        return;
    }
    serviceAdd(&profile.service.arrived[kind], 1);
    serviceAdd(&profile.service.accepted[kind], accepted);
}

bool df3ProfileServiceCheckpointNeeded(unsigned queue, bool failed)
{
    // Keep only the first-fault latch armed after the finite timing window.
    // Cost, receipt and scheduler-window collectors remain stopped.
    if (!profile.generation || (profile.service.snapshotMask & 4)) {
        return false;
    }
    if (failed) {
        return true;
    }
    if (!profile.active) {
        return false;
    }
    const unsigned mask = profile.service.snapshotMask;
    return (!(mask & 1) && queue >= 48) || (!(mask & 2) && queue >= 56);
}

void df3ProfileServiceCheckpoint(const df3ServiceSnapshot_t *snapshot)
{
    df3ProfileServiceCheckpointDetailed(snapshot, NULL);
}

void df3ProfileServiceCheckpointDetailed(const df3ServiceSnapshot_t *snapshot, const df3ProfileFault_t *fault)
{
    if (!snapshot || !df3ProfileServiceCheckpointNeeded(snapshot->queue, snapshot->failed)) {
        return;
    }
    df3ProfileService_t *s = &profile.service;
    for (unsigned i = 0; i < DF3_SERVICE_SNAPSHOTS; ++i) {
        if (s->snapshotMask & (1U << i)) {
            continue;
        }
        // A late fault may have a large queue; do not invent earlier threshold
        // observations from outside the timing interval.
        if (i < 2 && !profile.active) {
            continue;
        }
        if (i == 2 ? !snapshot->failed : snapshot->queue < 48 + 8 * i) {
            continue;
        }
        s->snapshot[i] = *snapshot;
        s->snapshotMask |= 1U << i;
        if (i == 2) {
            if (fault) {
                s->fault = *fault;
            }
            if (!s->fault.reasons) {
                s->fault.reasons = snapshot->overflow ? DF3_PROFILE_FAULT_QUEUE : DF3_PROFILE_FAULT_OTHER;
            }
            s->fault.observedUs = df3ProfileReadMicros();
            s->fault.timingActive = profile.active;
        }
    }
}

void df3ProfileBudget(unsigned phases, bool busy, uint32_t budgetTicks, uint32_t elapsedTicks, uint32_t nextTicks,
                      unsigned nextKey)
{
    if (!profile.active) {
        return;
    }
    df3ProfileBudget_t *b = &profile.budget;
    ++b->calls;
    b->phases += phases;
    b->yields += busy;
    b->zeroPhaseCalls += busy && !phases;
    if (phases > b->maxPhases) {
        b->maxPhases = phases;
    }
    if (elapsedTicks > b->maxElapsedTicks) {
        b->maxElapsedTicks = elapsedTicks;
    }
    if (nextTicks > b->maxNextTicks) {
        b->maxNextTicks = nextTicks;
        b->maxNextKey = nextKey;
    }
    if (elapsedTicks > budgetTicks) {
        ++b->overruns;
        if (elapsedTicks - budgetTicks > b->maxOverrunTicks) {
            b->maxOverrunTicks = elapsedTicks - budgetTicks;
        }
    }
}
#endif

void df3ProfilePoll(void)
{
    if (profile.active && (uint32_t)(df3ProfileReadMicros() - profile.startedUs) >= DF3_PROFILE_WINDOW_US) {
        df3ProfileStop();
    }
}

df3ProfileScope_t df3ProfileBegin(df3ProfileSection_e section)
{
    const df3ProfileScope_t scope = {.started = profile.active ? df3ProfileReadCycles() : 0,
                                     .generation = profile.active ? profile.generation : 0,
                                     .section = section};
    return scope;
}

void df3ProfileEnd(df3ProfileScope_t *scope)
{
    if (!scope->generation || !profile.active || scope->generation != profile.generation ||
        (unsigned)scope->section >= DF3_PROF_COUNT) {
        return;
    }
    const uint32_t end = df3ProfileReadCycles();
    const uint32_t elapsed = end - scope->started;
    df3ProfileRow_t *row = &profile.rows[scope->section];
    if (!row->calls || elapsed < row->minCycles) {
        row->minCycles = elapsed;
    }
    if (elapsed > row->maxCycles) {
        row->maxCycles = elapsed;
    }
    ++row->calls;
    row->totalCycles += elapsed;
    unsigned bin = 0;
    uint64_t upperCycles = profile.cyclesPerUs;
    while (bin < DF3_PROFILE_BINS - 1 && elapsed > upperCycles) {
        ++bin;
        upperCycles <<= 1;
    }
    ++row->histogram[bin];
    profile.bookkeepingCycles += (uint32_t)(df3ProfileReadCycles() - end);
}

void df3ProfileGauge(df3ProfileGauge_e id, uint64_t value)
{
    if (!profile.active || (unsigned)id >= DF3_PROF_GAUGE_COUNT) {
        return;
    }
    df3ProfileGauge_t *g = &profile.gauges[id];
    const uint32_t bounded = value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
    ++g->samples;
    g->last = bounded;
    g->sum += bounded;
    if (bounded > g->max) {
        g->max = bounded;
    }
}

void df3ProfileHealth(const df3ProfileHealth_t *health)
{
    if (!profile.active) {
        return;
    }
    if (!profile.hasHealth) {
        profile.first = *health;
        profile.hasHealth = true;
    }
    profile.last = *health;
}

const df3Profile_t *df3ProfileGet(void)
{
    return &profile;
}
const char *df3ProfileName(df3ProfileSection_e s)
{
    return names[s];
}
const char *df3ProfileGaugeName(df3ProfileGauge_e g)
{
    return gaugeNames[g];
}

uint32_t df3ProfileP95UpperUs(const df3ProfileRow_t *row)
{
    if (!row->calls) {
        return 0;
    }
    const uint32_t threshold = row->calls - row->calls / 20;
    uint32_t cumulative = 0;
    for (unsigned i = 0; i < DF3_PROFILE_BINS; ++i) {
        cumulative += row->histogram[i];
        if (cumulative >= threshold) {
            return i == DF3_PROFILE_BINS - 1 ? UINT32_MAX : 1U << i;
        }
    }
    return UINT32_MAX;
}
#endif
