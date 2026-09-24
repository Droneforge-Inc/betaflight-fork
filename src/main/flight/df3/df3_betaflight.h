#pragma once
#include "df3_estimator.h"
#include "df3_reference.h"
#include "df3_mlrs.h"
#include "df3_control.h"
#include "df3_state.h"
#include "df3_diagnostics.h"
#include <stdint.h>

#ifdef USE_DF3
#if defined(SITL) && defined(USE_DF3_SCHED_BENCH)
void df3BetaflightBenchTask(unsigned taskId, uint32_t intervalUs);
#endif
void df3BetaflightInit(void);
void df3BetaflightTick(void);
#ifdef USE_DF3_RESUMABLE
bool df3BetaflightFusionReady(void);
void df3BetaflightFusionTick(void);
#ifdef USE_DF3_BUDGETED_WORKER
unsigned df3BetaflightFusionMinTimeUs(void);
unsigned df3BetaflightFusionBudgetUs(void);
#endif
#endif
void df3BetaflightAccelerometer(void);
void df3BetaflightAttitude(void);
void df3BetaflightRange(void);
void df3BetaflightFlow(void);
void df3BetaflightMtfFrame(const uint8_t payload[20], uint8_t sequence);
const df3Estimate_t *df3BetaflightEstimate(void);
void df3BetaflightReferenceFrame(const uint8_t payload[DF3_REFERENCE_BYTES], uint32_t receivedUs);
bool df3BetaflightReference(df3Reference_t *out);
bool df3BetaflightAssistSelected(void);
bool df3BetaflightAssistActive(void);
const df3ControlOutput_t *df3BetaflightControl(void);
void df3BetaflightMlrsProfile(bool active, bool newSession);
bool df3BetaflightMlrsLegacyAllowed(void);
bool df3BetaflightMlrsControl(const uint8_t payload[DF3_MLRS_REFERENCE_BYTES], uint32_t receivedUs);
void df3BetaflightMlrsSnapshot(uint8_t payload[DF3_MLRS_SNAPSHOT_BYTES]);
void df3BetaflightStatePayload(uint8_t payload[DF3_STATE_BYTES]);
void df3BetaflightDiagnosticsPayload(uint8_t payload[DF3_DIAGNOSTICS_BYTES]);
void df3BetaflightTelemetryPoll(uint32_t now);
void df3BetaflightTelemetryQueued(uint32_t now, bool replacing);
void df3BetaflightUartSubmitted(uint32_t now, uint8_t type);
#if defined(USE_DF3_BLACKBOX) && defined(USE_BLACKBOX)
void df3BetaflightBlackbox(uint32_t nowUs, int32_t values[DF3_BLACKBOX_FIELD_COUNT]);
#endif
bool df3BetaflightCalibrationValid(float a0, float a1, float v0);
float df3BetaflightCalibrationHover(float a0, float a1, float voltage);
void df3BetaflightReloadCalibration(void);
// Reload persisted controller gains while disarmed; estimator state is preserved.
void df3BetaflightReloadGains(void);
bool df3BetaflightAssistMappingReady(void);
#endif
