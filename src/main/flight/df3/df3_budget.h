/* Opt-in continuation service policy. Cooperative safe points, not instruction
 * preemption. Clock units are caller ticks; the caller owns the deadline.
 * High-water observations are estimates, not certified worst-case bounds. */
#pragma once
#include "df3_resumable.h"
#include "df3_profile.h"
#include <string.h>

#ifdef USE_DF3_BUDGETED_WORKER
enum {
    DF3_BUDGET_KEYS = 80,
    DF3_BUDGET_PHASE_LIMIT = 256,
    DF3_BUDGET_BATCH_US = 100,
    DF3_BUDGET_WRAPPER_US = 2,
    DF3_BUDGET_ENTRY_US = 2
};
#ifdef USE_DF3_BUDGET_CYCLES
/* DWT wraps every ~25 s at 170 MHz. Unsigned differences are valid because
 * each callback/operation is far shorter than one wrap. No ns conversion or
 * 64-bit arithmetic is needed in the MCU's per-operation timing path. */
typedef uint32_t df3BudgetTicks_t;
#else
typedef uint64_t df3BudgetTicks_t; // native bSITL thread CPU nanoseconds
#endif
typedef df3BudgetTicks_t (*df3BudgetClockFn)(void *context);
typedef struct {
    df3BudgetTicks_t highWaterTicks[DF3_BUDGET_KEYS];
    uint32_t calls, phases, yields, overruns, phaseLimitHits;
    df3BudgetTicks_t lastBudgetTicks, lastElapsedTicks;
    uint32_t ticksPerUs;
    unsigned lastPhases;
} df3BudgetWorker_t;

static inline void df3BudgetInit(df3BudgetWorker_t *worker, uint32_t ticksPerUs)
{
    memset(worker, 0, sizeof(*worker));
    worker->ticksPerUs = ticksPerUs;
}

static inline df3BudgetTicks_t df3BudgetNextTicks(const df3BudgetWorker_t *worker, const df3FusionJob_t *job)
{
    const unsigned key = df3FusionPhaseKey(job);
    const df3BudgetTicks_t observed = key < DF3_BUDGET_KEYS ? worker->highWaterTicks[key] : 0;
    // Reserve a complete cold operation until this bucket has a measurement.
    // This is a bench assumption, not an STM32 WCET claim.
    const df3BudgetTicks_t estimate = observed ? observed : 20 * worker->ticksPerUs;
    // Include clock/policy overhead and observed short host timing excursions.
    // A phase is indivisible; a previously unseen larger cost is still reported
    // as an overrun rather than hidden or presented as true RTOS preemption.
    return estimate + estimate / 4 + 5 * worker->ticksPerUs;
}

/* One shared admission/service policy. A 100 us batch is a preferred quantum,
 * not a ceiling that can make the next indivisible operation impossible.
 * The caller's actual gyro-deadline window remains the hard offered limit.
 * Reserve loop-entry time as well as callback return time, so an exactly
 * admitted operation can still fit after the worker reads its first clock. */
static inline unsigned df3BudgetMinimumUs(const df3BudgetWorker_t *worker, const df3FusionJob_t *job,
                                          const df3Estimator_t *estimator)
{
    if (
#ifndef USE_DF3_MULTIRATE
        !job->busy ||
#endif
        estimator->failed || !estimator->initialized)
        return DF3_BUDGET_WRAPPER_US;
    const uint32_t rate = worker->ticksPerUs;
    if (!rate) {
        return DF3_BUDGET_BATCH_US;
    }
    const df3BudgetTicks_t next = df3BudgetNextTicks(worker, job);
    return (unsigned)(next / rate + (next % rate != 0)) + DF3_BUDGET_WRAPPER_US + DF3_BUDGET_ENTRY_US;
}

static inline unsigned df3BudgetCeilingUs(const df3BudgetWorker_t *worker, const df3FusionJob_t *job,
                                          const df3Estimator_t *estimator)
{
    const unsigned minimum = df3BudgetMinimumUs(worker, job, estimator);
    return minimum > DF3_BUDGET_BATCH_US ? minimum : DF3_BUDGET_BATCH_US;
}

static inline unsigned df3BudgetOfferUs(const df3BudgetWorker_t *worker, const df3FusionJob_t *job,
                                        const df3Estimator_t *estimator, unsigned availableUs)
{
    const unsigned minimum = df3BudgetMinimumUs(worker, job, estimator);
    if (availableUs < minimum) {
        return 0;
    }
    const unsigned ceiling = minimum > DF3_BUDGET_BATCH_US ? minimum : DF3_BUDGET_BATCH_US;
    return availableUs < ceiling ? availableUs : ceiling;
}

static inline df3BudgetTicks_t df3BudgetMathTicks(const df3BudgetWorker_t *worker, unsigned offeredUs)
{
    return offeredUs > DF3_BUDGET_WRAPPER_US
               ? (df3BudgetTicks_t)(offeredUs - DF3_BUDGET_WRAPPER_US) * worker->ticksPerUs
               : 0;
}

static inline void df3BudgetRun(df3BudgetWorker_t *worker, df3FusionJob_t *job, df3Estimator_t *estimator,
                                df3BudgetTicks_t budgetTicks, df3BudgetClockFn clock, void *context)
{
    worker->lastBudgetTicks = budgetTicks;
    worker->lastElapsedTicks = 0;
    worker->lastPhases = 0;
    if (job->busy && (estimator->failed || !estimator->initialized)) {
        // The continuation's existing fault branch only cancels the job and
        // invalidates publication. No matrix operation or clock is required.
        df3FusionResume(job, estimator);
        return;
    }
    if (!budgetTicks || !worker->ticksPerUs) {
        return;
    }
#ifndef USE_DF3_MULTIRATE
    if (!job->busy) {
        return;
    }
#else
    if (!job->busy && !df3FusionDue(job, estimator, job->lastNowUs)) {
        return;
    }
#endif
    ++worker->calls;
    const df3BudgetTicks_t started = clock(context);
    while (worker->lastPhases < DF3_BUDGET_PHASE_LIMIT && (job->busy
#ifdef USE_DF3_MULTIRATE
                                                           || df3FusionDue(job, estimator, job->lastNowUs)
#endif
                                                               )) {
        const df3BudgetTicks_t before = clock(context);
        const df3BudgetTicks_t elapsed = before - started;
        if (elapsed >= budgetTicks || df3BudgetNextTicks(worker, job) > budgetTicks - elapsed) {
            break;
        }
        const unsigned key = df3FusionPhaseKey(job);
#ifdef USE_DF3_PROFILE
        const unsigned kind = job->index < job->count ? job->batch[job->index].kind : DF3_SERVICE_NO_EVENT;
        const unsigned bucket =
#ifdef USE_DF3_MULTIRATE
            !job->busy ? 2 :
#endif
                       df3ProfileServiceBucket(job->phase, job->coreJob.phase, kind);
#endif
#ifdef USE_DF3_MULTIRATE
        if (!job->busy) {
            (void)df3FusionStart(job, estimator, job->lastNowUs);
        } else
#endif
            df3FusionResume(job, estimator);
        const df3BudgetTicks_t elapsedPhase = clock(context) - before;
#ifdef USE_DF3_PROFILE
        // Reuse the worker's measured phase interval; never time across a yield.
        df3ProfileServiceOperation(bucket, elapsedPhase > UINT32_MAX ? UINT32_MAX : elapsedPhase);
#endif
        if (key < DF3_BUDGET_KEYS && elapsedPhase > worker->highWaterTicks[key]) {
            worker->highWaterTicks[key] = elapsedPhase;
        }
        ++worker->lastPhases;
    }
    worker->lastElapsedTicks = clock(context) - started;
    worker->phases += worker->lastPhases;
    worker->overruns += worker->lastElapsedTicks > budgetTicks;
    worker->yields += job->busy;
    worker->phaseLimitHits += worker->lastPhases == DF3_BUDGET_PHASE_LIMIT && job->busy;
}
#endif
