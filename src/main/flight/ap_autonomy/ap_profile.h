#pragma once

#include "backend/profile.h"
#include <stdbool.h>

#ifdef USE_AP_PROFILE
#define AP_PROFILE_BINS 10
typedef struct {
    uint64_t totalCycles;
    uint32_t calls, minCycles, maxCycles;
    uint32_t histogram[AP_PROFILE_BINS]; // <= 8,16,...,2048 us, then overflow
} apProfileRow_t;

typedef struct {
    bool active;
    uint32_t startedUs, stoppedUs, cyclesPerUs;
    apProfileRow_t rows[AP_PROF_COUNT];
} apProfile_t;

void apProfileStart(void);
void apProfileStop(void);
void apProfileSuspend(void);
void apProfileResume(void);
const apProfile_t *apProfileGet(void);
const char *apProfileName(apProfileSection_e section);
#endif
