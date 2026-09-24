#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "config/feature.h"
#include "io/serial.h"
#include "rx/crsf.h"
#include "telemetry/crsf.h"

static bool telemetry, negotiated;
static unsigned frames;
static uint8_t queued[64];
static unsigned queuedLength;
const uint32_t baudRates[BAUD_COUNT] = {0};
bool featureIsEnabled(uint32_t mask) { return mask == FEATURE_TELEMETRY && telemetry; }
bool crsfRxUseNegotiatedBaud(void) { return negotiated; }
void crsfRxUpdateBaudrate(uint32_t baud) { (void)baud; }
void crsfRxWriteTelemetryData(const void *p, int n) {
    assert(n > 0 && n <= 64); memcpy(queued, p, n); queuedLength = n;
}
void crsfRxSendTelemetryData(void) { if (queuedLength) { ++frames; queuedLength = 0; } }
void df3BetaflightTelemetryQueued(uint32_t now, bool busy) { (void)now; (void)busy; }
uint32_t micros(void) { return 0; }
int main(void) {
    speedNegotiationProcess(1000000);
    assert(frames == 0); // Recovery: both disabled must emit no bytes.
    telemetry = true;
    speedNegotiationProcess(2000000);
    assert(frames == 1 && queued[2] == CRSF_FRAMETYPE_DEVICE_PING);
    negotiated = true;
    speedNegotiationProcess(3000000);
    assert(frames == 2 && queued[2] == CRSF_FRAMETYPE_DEVICE_PING);
    telemetry = false;
    speedNegotiationProcess(4000000);
    assert(frames == 3 && queued[2] == CRSF_FRAMETYPE_HEARTBEAT);
    negotiated = false;
    speedNegotiationProcess(5000000);
    assert(frames == 3); // Restoring quiet settings stops subsequent discovery.
    puts("CRSF startup UART gating: all four combinations passed");
}
