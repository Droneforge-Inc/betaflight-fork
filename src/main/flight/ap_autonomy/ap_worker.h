#pragma once
#include <stdbool.h>
#include <stdint.h>

#define AP_WORKER_MIN_US 100u
#define AP_WORKER_MAX_US 120u

// A single AP transaction owns its stack and DAL until it finishes. No other
// caller may enter the backend while it is suspended.
void apWorkerInit(void (*transaction)(void));
bool apWorkerResume(unsigned budgetUs);
bool apWorkerBusy(void);
uint32_t apWorkerStackFree(void);
typedef struct {
    uint32_t slices, jobs, overBudgetSlices, maxSliceCycles, maxCheckpointCycles, maxJobUs;
    uint32_t maxGapFromPc, maxGapToPc;
} apWorkerStats_t;
const apWorkerStats_t *apWorkerStats(void);
void apWorkerResetStats(void);
