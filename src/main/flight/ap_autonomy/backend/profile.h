#pragma once

#include <stdint.h>

typedef enum {
    AP_PROF_TASK,
    AP_PROF_FLOW,
    AP_PROF_EKF_PREP,
    AP_PROF_EKF_CORE,
    AP_PROF_EKF_READBACK,
    AP_PROF_CONTROL,
    AP_PROF_PROBE,
    AP_PROF_EKF_COVARIANCE,
    AP_PROF_EKF_POSVEL,
    AP_PROF_EKF_OPTFLOW,
    AP_PROF_COUNT
} apProfileSection_e;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USE_AP_PROFILE
// Cooperative scheduler context only. Elapsed cycles include interrupts.
// Task/child scopes overlap; do not sum parent and child measurements.
uint32_t apProfileBegin(apProfileSection_e section);
void apProfileEnd(apProfileSection_e section, uint32_t started);
#else
static inline uint32_t apProfileBegin(apProfileSection_e section)
{
    (void)section;
    return 0;
}
static inline void apProfileEnd(apProfileSection_e section, uint32_t started)
{
    (void)section;
    (void)started;
}
#endif

#ifdef __cplusplus
}
#endif
