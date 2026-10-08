#pragma once

// A checkpoint may suspend only the AP worker. Its stack, estimator transaction
// and DAL snapshot stay private until it resumes; completed outputs are published
// by the adapter. Calls outside a running worker must return without suspending.
#ifdef USE_AP_WORKER
#ifdef __cplusplus
extern "C" {
#endif
void apAutonomyCheckpoint(void);
#ifdef __cplusplus
}
#endif
#else
static inline void apAutonomyCheckpoint(void) {}
#endif
