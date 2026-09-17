#pragma once
#include "df3_protocol.h"
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Stable array indices. One means the named condition occurred; never a command. */
enum {
    DF3_DIAG_IMU_STALE,
    DF3_DIAG_RANGE_STALE,
    DF3_DIAG_FLOW_UNUSABLE,
    DF3_DIAG_TICK_LATE,
    DF3_DIAG_FUSION_STALE,
    DF3_DIAG_ESTIMATOR_INVALID,
    DF3_DIAG_QUEUE_OVERFLOW,
    DF3_DIAG_REFERENCE_MISSING,
    DF3_DIAG_CONTROL_FAULT,
    DF3_DIAG_NATIVE_FAILSAFE,
    DF3_DIAG_PERMIT_LOST,
    DF3_DIAG_TELEMETRY_LATE,
    DF3_DIAG_UART_LATE,
    DF3_DIAG_OPTRANGE_UART_LATE,
    DF3_DIAG_REFERENCE_REJECTED,
    DF3_DIAG_TELEMETRY_BUFFER_BLOCKED,
    DF3_DIAG_COUNT
};

typedef struct {
    bool armed, selected, permit, imuFresh, rangeFresh, flowFresh, fusionFresh;
    bool estimatorValid, referenceValid, controllerFault, nativeFailsafe;
    uint32_t overflow, rejected;
} df3DiagnosticsInput_t;
typedef struct {
    uint32_t lastTickUs, lastTelemetryUs, lastUartUs, lastOptrangeUs, pendingUs;
    uint32_t startedUs, overflow, rejected;
    uint16_t current, latched, checked, latchedChecked, episode, sequence, events;
    bool armed, monitoring, pending;
    uint8_t context;
} df3FaultDiagnostics_t;

void df3DiagnosticsUpdate(df3FaultDiagnostics_t *d, uint32_t now, const df3DiagnosticsInput_t *in);
void df3DiagnosticsTelemetryPoll(df3FaultDiagnostics_t *d, uint32_t now);
void df3DiagnosticsTelemetryQueued(df3FaultDiagnostics_t *d, uint32_t now, bool replacing);
/* Reports return from serialWriteBuf, not physical wire or radio acknowledgment. */
void df3DiagnosticsUartSubmitted(df3FaultDiagnostics_t *d, uint32_t now, uint8_t type);
void df3DiagnosticsEncode(df3FaultDiagnostics_t *d, uint32_t sourceMs, uint8_t payload[DF3_DIAGNOSTICS_BYTES]);
#ifdef __cplusplus
}
#endif
