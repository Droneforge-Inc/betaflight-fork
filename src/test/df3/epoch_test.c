#include "platform.h"
#include "drivers/persistent.h"
#include "flight/df3/df3_epoch.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
RNG_TypeDef fakeRng;
RCC_TypeDef fakeRcc;
static uint32_t backup[32], now;
uint32_t micros(void) { return now; }
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *h, unsigned id) { (void)h; assert(id < 32); return backup[id]; }
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *h, unsigned id, uint32_t value) { (void)h; assert(id < 32); backup[id] = value; }
int main(void)
{
    // No estimator or sensor callbacks: even an invalid-state boot gets identity.
    fakeRng.SR = RNG_SR_DRDY; fakeRng.DR = 0x1234;
    persistentObjectInit();
    const uint16_t firstBoot = df3EpochNext();
    assert(firstBoot == 0x1234);
    assert(!fakeRcc.AHB2ENR && !fakeRng.CR);
    assert(backup[PERSISTENT_OBJECT_DF3_EPOCH] == firstBoot);
    // CLI exit soft reboot preserves identity state and allocates a new epoch.
    fakeRcc.CSR = RCC_CSR_SFTRSTF;
    fakeRng.DR = 0xbeef;
    persistentObjectInit();
    assert(df3EpochNext() == firstBoot + 1);
    // Estimator initialization also persists its advance, preventing next-boot reuse.
    assert(df3EpochNext() == firstBoot + 2);
    persistentObjectInit();
    assert(df3EpochNext() == firstBoot + 3);
    // Watchdog/pin reset clears legacy reset state, but keeps telemetry identity.
    fakeRcc.CSR = 0;
    backup[PERSISTENT_OBJECT_SERIALRX_BAUD] = 420000;
    persistentObjectInit();
    assert(!backup[PERSISTENT_OBJECT_SERIALRX_BAUD]);
    assert(df3EpochNext() == firstBoot + 4);
    backup[PERSISTENT_OBJECT_DF3_EPOCH] = 65535;
    assert(df3EpochNext() == 1);
    // Cold backup loss draws fresh entropy, not the deterministic warm-boot ID.
    memset(backup, 0, sizeof(backup));
    persistentObjectInit();
    assert(df3EpochNext() == 0xbeef);
    // Bounded timeout/error fallback still allocates nonzero identity.
    memset(backup, 0, sizeof(backup)); fakeRng.SR = 0;
    persistentObjectInit();
    assert(df3EpochNext() == 1);
    memset(backup, 0, sizeof(backup)); fakeRng.SR = RNG_SR_DRDY | RNG_SR_SECS; now = 42;
    persistentObjectInit();
    assert(df3EpochNext() == 42);
    puts("DF3 epoch: sensor-independent cold/soft/pin restart, persistence, RNG and wrap passed");
}
