#include "df3_diagnostics.h"
enum { DF3_DIAG_TICK_GAP_US = 20000, DF3_DIAG_TELEMETRY_GAP_US = 100000, DF3_DIAG_UART_GAP_US = 300000 };
#define FLAG(n) ((uint16_t)(1u << (n)))
static bool late(uint32_t now, uint32_t then, uint32_t limit)
{
    return now - then > limit;
}
static void event(df3FaultDiagnostics_t *d, unsigned bit)
{
    if (d->monitoring && d->armed) {
        d->events |= FLAG(bit);
        d->latched |= FLAG(bit);
        d->latchedChecked |= FLAG(bit);
    }
}
void df3DiagnosticsTelemetryPoll(df3FaultDiagnostics_t *d, uint32_t now)
{
    if (d->lastTelemetryUs && late(now, d->lastTelemetryUs, DF3_DIAG_TELEMETRY_GAP_US)) {
        event(d, DF3_DIAG_TELEMETRY_LATE);
    }
    d->lastTelemetryUs = now;
}
void df3DiagnosticsTelemetryQueued(df3FaultDiagnostics_t *d, uint32_t now, bool replacing)
{
    if (replacing) {
        event(d, DF3_DIAG_TELEMETRY_BUFFER_BLOCKED);
    }
    if (!d->pending) {
        d->pendingUs = now;
    }
    d->pending = true;
}
void df3DiagnosticsUartSubmitted(df3FaultDiagnostics_t *d, uint32_t now, uint8_t type)
{
    if (d->lastUartUs && late(now, d->lastUartUs, DF3_DIAG_UART_GAP_US)) {
        event(d, DF3_DIAG_UART_LATE);
    }
    d->lastUartUs = now;
    d->pending = false;
    if (type == DF3_OPTRANGE_CRSF_TYPE || type == 0xea) {
        if (d->lastOptrangeUs && late(now, d->lastOptrangeUs, DF3_DIAG_UART_GAP_US)) {
            event(d, DF3_DIAG_OPTRANGE_UART_LATE);
        }
        d->lastOptrangeUs = now;
    }
}
void df3DiagnosticsUpdate(df3FaultDiagnostics_t *d, uint32_t now, const df3DiagnosticsInput_t *in)
{
    if (in->armed && !d->armed) {
        d->latched = d->latchedChecked = d->events = 0;
        d->monitoring = false;
        if (++d->episode == 0) {
            ++d->episode;
        }
        d->overflow = in->overflow;
        d->rejected = in->rejected;
    }
    d->armed = in->armed;
    if (in->armed && in->selected && in->permit && !d->monitoring) {
        d->monitoring = true;
        d->startedUs = now;
        d->overflow = in->overflow;
        d->rejected = in->rejected;
    }
    if (!in->armed) {
        d->monitoring = false;
    }
    d->context = (in->armed ? DF3_DIAGNOSTICS_ARMED : 0) | (in->selected ? DF3_DIAGNOSTICS_SELECTED : 0) |
                 (in->permit ? DF3_DIAGNOSTICS_PERMIT : 0) | (d->monitoring ? DF3_DIAGNOSTICS_MONITORING : 0);
    d->current = d->checked = 0;
    if (d->monitoring && in->armed && in->selected) {
        const bool checks[DF3_DIAG_COUNT] = {
            !in->imuFresh,
            !in->rangeFresh,
            !in->flowFresh,
            d->lastTickUs && late(now, d->lastTickUs, DF3_DIAG_TICK_GAP_US),
            !in->fusionFresh,
            !in->estimatorValid,
            in->overflow != d->overflow,
            !in->referenceValid,
            in->controllerFault,
            in->nativeFailsafe,
            !in->permit,
            late(now, d->lastTelemetryUs ? d->lastTelemetryUs : d->startedUs, DF3_DIAG_TELEMETRY_GAP_US),
            late(now, d->lastUartUs ? d->lastUartUs : d->startedUs, DF3_DIAG_UART_GAP_US),
            late(now, d->lastOptrangeUs ? d->lastOptrangeUs : d->startedUs, DF3_DIAG_UART_GAP_US),
            in->rejected != d->rejected,
            d->pending && late(now, d->pendingUs, DF3_DIAG_TELEMETRY_GAP_US)};
        d->checked = UINT16_MAX;
        for (unsigned i = 0; i < DF3_DIAG_COUNT; ++i) {
            if (checks[i]) {
                d->current |= FLAG(i);
            }
        }
        d->current |= d->events;
        d->latched |= d->current;
        d->latchedChecked |= d->checked;
    }
    d->events = 0;
    d->overflow = in->overflow;
    d->rejected = in->rejected;
    d->lastTickUs = now;
    /* Retain this flight's latch through disarm; reset only on the next arm. */
}
void df3DiagnosticsEncode(df3FaultDiagnostics_t *d, uint32_t sourceMs, uint8_t p[DF3_DIAGNOSTICS_BYTES])
{
    p[DF3_WIRE_VERSION] = DF3_PROTOCOL_VERSION;
    p[DF3_WIRE_FLAGS] = d->context;
    df3WriteU16Be(p + DF3_WIRE_EPOCH, d->episode);
    df3WriteU16Be(p + DF3_WIRE_SEQUENCE, d->sequence++);
    df3WriteU16Be(p + DF3_WIRE_SOURCE_MS, (uint16_t)(sourceMs >> 16));
    df3WriteU16Be(p + DF3_WIRE_SOURCE_MS + 2, (uint16_t)sourceMs);
    df3WriteU16Be(p + DF3_WIRE_DIAGNOSTICS_CURRENT, d->current);
    df3WriteU16Be(p + DF3_WIRE_DIAGNOSTICS_LATCHED, d->latched);
    df3WriteU16Be(p + DF3_WIRE_DIAGNOSTICS_CHECKED, d->checked);
    df3WriteU16Be(p + DF3_WIRE_DIAGNOSTICS_LATCHED_CHECKED, d->latchedChecked);
}
