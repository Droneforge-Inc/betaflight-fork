#include "platform.h"
#if defined(USE_AP_AUTONOMY) && defined(USE_AP_WORKER)
#include "ap_worker.h"
#include "ap_profile.h"
#include "backend/checkpoint.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include <string.h>
#ifdef SITL
#include <time.h>
#include <ucontext.h>
#define WORKER_STACK_BYTES (64 * 1024)
#else
#define WORKER_STACK_BYTES 6144
extern void apContextSwitch(uint32_t **from, uint32_t *to);
static uint32_t *mainSp, *workerSp;
#endif

#define WORKER_GUARD_BYTES 128u
#define WORKER_CHECKPOINT_RESERVE_US 80u

static uint32_t apWorkerStack[WORKER_STACK_BYTES / sizeof(uint32_t)] __attribute__((aligned(8)));
static void (*runTransaction)(void);
static bool initialized, busy, running, failed;
static uint32_t deadline, checkpointCycles, checkpointPc, cyclesPerUs, jobStartedUs;
static apWorkerStats_t stats;
#ifdef SITL
static ucontext_t mainContext, workerContext;
#endif

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

static void suspend(void)
{
    running = false;
#ifdef USE_AP_PROFILE
    apProfileSuspend();
#endif
#ifdef SITL
    if (swapcontext(&workerContext, &mainContext) != 0) {
        __builtin_trap();
    }
#else
    apContextSwitch(&workerSp, mainSp);
#endif
    running = true;
}

static void entry(void)
{
    for (;;) {
        const uint32_t start = apProfileBegin(AP_PROF_TASK);
        runTransaction();
        apProfileEnd(AP_PROF_TASK, start);
        const uint32_t elapsed = (uint32_t)(micros() - jobStartedUs);
        if (elapsed > stats.maxJobUs) {
            stats.maxJobUs = elapsed;
        }
        ++stats.jobs;
        checkpointPc = 0;
        busy = false;
        suspend();
    }
}

void apWorkerInit(void (*transaction)(void))
{
    runTransaction = transaction;
    memset(apWorkerStack, 0xa5, sizeof(apWorkerStack));
#ifdef SITL
    cyclesPerUs = 1000;
    if (getcontext(&workerContext) != 0) {
        failed = true;
        return;
    }
    workerContext.uc_stack.ss_sp = (uint8_t *)apWorkerStack + WORKER_GUARD_BYTES;
    workerContext.uc_stack.ss_size = sizeof(apWorkerStack) - WORKER_GUARD_BYTES;
    workerContext.uc_link = &mainContext;
    makecontext(&workerContext, entry, 0);
#else
    cyclesPerUs = clockMicrosToCycles(1);
    // ap_context_m4f.S restores FPSCR, r4-r11, LR, s16-s31 (104 bytes).
    workerSp = apWorkerStack + WORKER_STACK_BYTES / sizeof(uint32_t) - 26;
    memset(workerSp, 0, 104);
    __asm__ volatile ("vmrs %0, fpscr" : "=r" (workerSp[0]));
    workerSp[9] = (uint32_t)entry;
#endif
    initialized = true;
}

bool apWorkerResume(unsigned budgetUs)
{
    const uint32_t start = cycles();
    if (!initialized || failed) {
        return false;
    }
    if (budgetUs < (busy ? AP_WORKER_MIN_US : AP_WORKER_MAX_US)) {
        return true;
    }
    if (budgetUs > AP_WORKER_MAX_US) {
        budgetUs = AP_WORKER_MAX_US;
    }
    if (!busy) {
        busy = true;
        jobStartedUs = micros();
    }
    // Steady-state G4 capture found a 72 us maximum checkpoint gap. Reserve
    // that plus 8 us for return/accounting; startup and interrupt peaks still
    // require hardware validation. Yielding cannot interrupt a block itself.
    deadline = start + (budgetUs - WORKER_CHECKPOINT_RESERVE_US) * cyclesPerUs;
    checkpointCycles = start;
#ifdef USE_AP_PROFILE
    apProfileResume();
#endif
    running = true;
#ifdef SITL
    if (swapcontext(&mainContext, &workerContext) != 0) {
        failed = true;
    }
#else
    apContextSwitch(&mainSp, workerSp);
#endif
    running = false;
    // Keep the full 128-byte guard, but check aligned words. The former byte
    // loop consumed several microseconds outside the reported slice duration.
    for (unsigned i = 0; i < WORKER_GUARD_BYTES / sizeof(apWorkerStack[0]); ++i) {
        if (apWorkerStack[i] != 0xa5a5a5a5u) {
            failed = true;
        }
    }
    ++stats.slices;
    const uint32_t elapsed = cycles() - start;
    if (elapsed > budgetUs * cyclesPerUs) {
        ++stats.overBudgetSlices;
    }
    if (elapsed > stats.maxSliceCycles) {
        stats.maxSliceCycles = elapsed;
    }
    return !failed;
}

__attribute__((noinline)) void apAutonomyCheckpoint(void)
{
    if (!running) {
        return;
    }
    const uint32_t now = cycles();
    const uint32_t pc = (uint32_t)(uintptr_t)__builtin_return_address(0);
    const uint32_t gap = now - checkpointCycles;
    if (gap > stats.maxCheckpointCycles) {
        stats.maxCheckpointCycles = gap;
        stats.maxGapFromPc = checkpointPc;
        stats.maxGapToPc = pc;
    }
    checkpointCycles = now;
    checkpointPc = pc;
    if ((int32_t)(now - deadline) >= 0) {
        suspend();
    }
}

bool apWorkerBusy(void) { return busy; }
const apWorkerStats_t *apWorkerStats(void) { return &stats; }
void apWorkerResetStats(void) { memset(&stats, 0, sizeof(stats)); }
uint32_t apWorkerStackFree(void)
{
    const uint8_t *stack = (const uint8_t *)apWorkerStack;
    unsigned bytes = 0;
    while (bytes < sizeof(apWorkerStack) && stack[bytes] == 0xa5) {
        ++bytes;
    }
    return bytes;
}
#endif
