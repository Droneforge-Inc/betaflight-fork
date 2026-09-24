// Include the real producer so the fixture can seed the scheduler's private state.
#include "flight/df3/df3_betaflight.c"
#include <assert.h>
#include <stdio.h>
#include "pg/rx.h"

static uint32_t clockUs = 1000000;
static quaternion_t nativeQ;
void getQuaternion(quaternion_t *q) { *q = nativeQ; }
bool df3EstimatorEnqueue(df3Estimator_t *e, const df3Event_t *event) { (void)e; (void)event; return false; }
uint8_t atomic_BASEPRI, armingFlags;
uint16_t flightModeFlags;
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
rxConfig_t rxConfig_System;
df3Config_t df3Config_System;
uint32_t micros(void) { return clockUs; }
uint16_t df3EpochNext(void) { return 1; }
bool failsafeIsActive(void) { return false; }
bool isFlipOverAfterCrashActive(void) { return false; }
bool isLaunchControlActive(void) { return false; }
uint32_t getBatterySampleAgeUs(uint32_t now) { (void)now; return UINT32_MAX; }
uint8_t getBatteryCellCount(void) { return 0; }
uint16_t getBatteryVoltage(void) { return 0; }
int32_t getAmperage(void) { return 0; }
int32_t getMAhDrawn(void) { return 0; }
uint8_t calculateBatteryPercentageRemaining(void) { return 0; }

int main(void)
{
    uint8_t p[66], report[20] = {0};
    booted = true;
    df3BetaflightMlrsSnapshot(p);
    assert(p[59] == 255 && !(p[57] & 12)); // No sensor report.

    df3WriteU32Le(report, 1000);
    report[16] = 15; // Fresh complete report, range and flow status invalid.
    df3BetaflightMtfFrame(report, 1);
    df3BetaflightMlrsSnapshot(p);
    assert(p[59] == 0 && !(p[57] & 12));
    assert(df3ReadU16Le(p + 35) == UINT16_MAX);
    assert(p[43] == 15 && p[44] == 0 && !(p[1] & DF3_STATE_VALID));

    clockUs += 20000;
    df3WriteU32Le(report, 1020);
    df3WriteU32Le(report + 4, 20);
    report[10] = 1; // Ground hardware case: valid 20 mm range, no flow lock.
    df3BetaflightMtfFrame(report, 2);
    df3BetaflightMlrsSnapshot(p);
    assert(p[59] == 0 && (p[57] & 12) == 4);
    assert(df3ReadU16Le(p + 35) == 20 && !(p[1] & DF3_STATE_VALID));

    clockUs += 20000;
    df3WriteU32Le(report, 1040);
    report[17] = 1;
    df3BetaflightMtfFrame(report, 3);
    df3BetaflightMlrsSnapshot(p);
    assert(p[59] == 0 && (p[57] & 12) == 12);

    clockUs += 100000;
    df3BetaflightMtfFrame(report, 3); // Duplicate must not refresh acquisition age.
    df3BetaflightMlrsSnapshot(p);
    assert(p[59] == 10 && (p[57] & 12) == 12);
    clockUs += 50001;
    df3BetaflightMlrsSnapshot(p);
    assert(p[59] == 255 && !(p[57] & 12));
    assert(df3ReadU16Le(p + 35) == UINT16_MAX);
    // Exercise the actual native attitude adapter, including its FRD conversion.
    // Native quaternion Euler(0.2,0.3,0.4) maps to FRD(0.2,-0.3,-0.4).
    const float cr = cosf(.1f), sr = sinf(.1f), cp = cosf(.15f), sp = sinf(.15f);
    const float cy = cosf(.2f), sy = sinf(.2f);
    nativeQ = (quaternion_t){.w = cr * cp * cy + sr * sp * sy,
                             .x = sr * cp * cy - cr * sp * sy,
                             .y = cr * sp * cy + sr * cp * sy,
                             .z = cr * cp * sy - sr * sp * cy};
    df3BetaflightAttitude();
    df3BetaflightMlrsSnapshot(p);
    assert(p[58] == 0 && !(p[1] & (DF3_STATE_VALID | DF3_STATE_VERTICAL_VALID)));
    assert((int16_t)df3ReadU16Le(p + 29) == 200);
    assert((int16_t)df3ReadU16Le(p + 31) == -300);
    assert((int16_t)df3ReadU16Le(p + 33) == -400);
    clockUs += 40000;
    df3BetaflightMlrsSnapshot(p);
    assert(p[58] == 4); // Polling snapshots never renews native attitude time.

    estimate.valid = estimate.verticalReferenceValid = true;
    estimate.timeUs = estimate.covarianceTimeUs = clockUs;
    estimate.x[DF3_Q] = 1;
    estimate.x[0] = 1.23f;
    df3BetaflightMlrsSnapshot(p);
    assert((p[1] & 3) == 3 && p[58] == 0);
    assert(df3ReadU16Le(p + 11) == 123 && df3ReadU16Le(p + 29) == 0);

    clockUs += 160000;
    df3BetaflightAttitude();
    df3BetaflightMlrsSnapshot(p);
    assert(!(p[1] & 3) && p[58] == 0); // Stale fusion falls back to fresh attitude.
    assert((int16_t)df3ReadU16Le(p + 31) == -300);
    clockUs += 150001;
    df3BetaflightMlrsSnapshot(p);
    assert(p[58] == 255 && !(p[1] & 3));
    nativeQ = (quaternion_t){0};
    df3BetaflightAttitude();
    df3BetaflightMlrsSnapshot(p);
    assert(p[58] == 255 && !(p[1] & 3)); // Invalid native quaternion is unavailable.
    // Native presets keep their AUX3-high turtle assignment; no DF3 overlay.
    memset(modeActivationConditions_SystemArray, 0, sizeof(modeActivationConditions_SystemArray));
    const modeActivationCondition_t turtle = {.modeId = BOXFLIPOVERAFTERCRASH,
        .auxChannelIndex = 2, .range = {.startStep = 32, .endStep = 48}};
    modeActivationConditions_SystemArray[3] = turtle;
    configureAssist();
    assert(!isModeActivationConditionPresent(BOXFLIGHTASSIST));
    assert(!memcmp(&modeActivationConditions_SystemArray[3], &turtle, sizeof(turtle)));
    assert(!df3BetaflightAssistMappingReady());
    df3BetaflightMlrsProfile(true, true);
    assert(!df3BetaflightAssistMappingReady()); // Negotiation alone is insufficient.
    uint8_t command[32] = {1, DF3_MLRS_AUTONOMY};
    df3WriteU16Le(command + 2, 7);
    df3WriteU32Le(command + 6, clockUs / 1000);
    df3WriteU16Le(command + 30, 200);
    assert(df3BetaflightMlrsControl(command, clockUs));
    assert(df3BetaflightAssistMappingReady() && df3BetaflightAssistSelected());
    df3BetaflightMlrsProfile(false, false);
    assert(!df3BetaflightAssistMappingReady());

    // Legacy default remains available where AUX3 high is unused.
    modeActivationConditions_SystemArray[3].range = (channelRange_t){0, 16};
    configureAssist();
    assert(isModeActivationConditionPresent(BOXFLIGHTASSIST));
    assert(df3BetaflightAssistMappingReady());
    assert(modeActivationConditions_SystemArray[3].modeId == BOXFLIPOVERAFTERCRASH);
    assert(modeActivationConditions_SystemArray[3].range.endStep == 16);

    // Ten minutes with either sign of 500 ppm sensor/FC clock mismatch.
    // Fresh reports must stay fresh; delivery stalls and duplicates must not.
    for (int skewUs = -10; skewUs <= 10; skewUs += 20) {
        memset(&mtf, 0, sizeof(mtf));
        clockAligned = false;
        uint32_t sourceMs = UINT32_MAX - 999; // Cross the sensor clock wrap.
        df3WriteU32Le(report, sourceMs);
        df3BetaflightMtfFrame(report, 0);
        uint64_t previousEnd = mtf.endUs;
        for (unsigned i = 1; i <= 30000; ++i) {
            clockUs += 20000 + skewUs;
            sourceMs += 20;
            df3WriteU32Le(report, sourceMs);
            df3BetaflightMtfFrame(report, (uint8_t)i);
            assert(mtf.endUs > previousEnd && mtf.endUs <= mtf.receivedUs);
            assert(mtf.receivedUs - mtf.endUs < 1000);
            previousEnd = mtf.endUs;
        }
        clockUs += 120000;
        sourceMs += 20;
        df3WriteU32Le(report, sourceMs);
        df3BetaflightMtfFrame(report, (uint8_t)30001);
        assert(mtf.receivedUs - mtf.endUs > 99000); // Real delivery delay remains.
        previousEnd = mtf.endUs;
        const uint32_t generation = mtf.generation;
        clockUs += 20000;
        df3BetaflightMtfFrame(report, (uint8_t)30001);
        assert(mtf.endUs == previousEnd && mtf.generation == generation);
        clockUs += 40000;
        df3BetaflightMlrsSnapshot(p);
        assert(p[59] == 255 && !(p[57] & 12)); // No report still expires.
        sourceMs += 2000;
        clockUs += 2000000;
        df3WriteU32Le(report, sourceMs);
        df3BetaflightMtfFrame(report, 1);
        assert(!mtf.timingValid && mtf.endUs == mtf.receivedUs);
    }
    puts("DF3 actual snapshot: sensor presence, native attitude and explicit mLRS selection passed");
    return 0;
}
