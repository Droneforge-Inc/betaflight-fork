#include "platform.h"
#if defined(USE_AP_AUTONOMY) && defined(USE_AP_PROFILE)
#include "ap_profile.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include <string.h>
#ifdef SITL
#include <time.h>
#endif

// All hooks run on the cooperative scheduler. No interrupts, allocation, or
// serial output are added to measured work. Nested sections are inclusive.
static apProfile_t profile;
static bool openSections[AP_PROF_COUNT], suspended;
static uint32_t suspendedAt, excludedCycles;
static const char *const names[AP_PROF_COUNT] = {
    "task", "flow", "ekf_prep", "ekf_core", "ekf_readback", "controller", "empty_probe",
    "ekf_covariance", "ekf_posvel", "ekf_optflow"
};

static uint32_t cycles(void)
{
#ifdef SITL
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint32_t)((uint64_t)now.tv_sec * 1000000000 + now.tv_nsec);
#else
    return getCycleCounter();
#endif
}

void apProfileStart(void)
{
    memset(&profile, 0, sizeof(profile));
    memset(openSections, 0, sizeof(openSections));
#ifdef SITL
    profile.cyclesPerUs = 1000;
#else
    profile.cyclesPerUs = clockMicrosToCycles(1);
#endif
    profile.startedUs = micros();
    profile.active = profile.cyclesPerUs != 0;
    for (unsigned i = 0; i < 32; ++i) {
        const uint32_t start = apProfileBegin(AP_PROF_PROBE);
        apProfileEnd(AP_PROF_PROBE, start);
    }
}

void apProfileStop(void)
{
    if (profile.active) {
        profile.stoppedUs = micros();
        profile.active = false;
    }
}

uint32_t apProfileBegin(apProfileSection_e section)
{
    if (section == AP_PROF_TASK && profile.active &&
        (uint32_t)(micros() - profile.startedUs) >= 20000000) {
        apProfileStop();
    }
    openSections[section] = profile.active;
    return profile.active ? cycles() - excludedCycles : 0;
}

void apProfileEnd(apProfileSection_e section, uint32_t start)
{
    if (!profile.active || !openSections[section]) {
        return;
    }
    openSections[section] = false;
    const uint32_t elapsed = cycles() - excludedCycles - start;
    apProfileRow_t *row = &profile.rows[section];
    if (!row->calls || elapsed < row->minCycles) {
        row->minCycles = elapsed;
    }
    if (elapsed > row->maxCycles) {
        row->maxCycles = elapsed;
    }
    ++row->calls;
    row->totalCycles += elapsed;
    unsigned bin = 0;
    uint32_t upper = 8 * profile.cyclesPerUs;
    while (bin < AP_PROFILE_BINS - 1 && elapsed > upper) {
        ++bin;
        upper *= 2;
    }
    ++row->histogram[bin];
}

const apProfile_t *apProfileGet(void) { return &profile; }
const char *apProfileName(apProfileSection_e section) { return names[section]; }

void apProfileSuspend(void)
{
    suspendedAt = cycles();
    suspended = true;
}

void apProfileResume(void)
{
    if (suspended) {
        excludedCycles += cycles() - suspendedAt;
        suspended = false;
    }
}
#endif
