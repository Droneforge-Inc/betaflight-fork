#include "platform.h"
#ifdef USE_DF3
#include "df3_epoch.h"
#ifdef SITL
#include <time.h>
#else
#include "drivers/persistent.h"
#include "drivers/time.h"
#endif

static uint16_t bootSeed(void)
{
#ifdef SITL
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint32_t seed = (uint32_t)now.tv_nsec ^ (uint32_t)now.tv_sec;
#else
    uint32_t seed = micros() ^ U_ID_0 ^ U_ID_1 ^ U_ID_2;
#if defined(STM32G4) && defined(RNG)
    // RNG shares the already configured USB 48 MHz source. Do not change it.
    const uint32_t enabled = RCC->AHB2ENR & RCC_AHB2ENR_RNGEN;
    __HAL_RCC_RNG_CLK_ENABLE();
    const uint32_t control = RNG->CR;
    RNG->CR = RNG_CR_RNGEN;
    for (unsigned attempt = 0; attempt < 10000; ++attempt) {
        const uint32_t status = RNG->SR;
        if (status & (RNG_SR_CECS | RNG_SR_SECS)) break;
        if (status & RNG_SR_DRDY) {
            seed ^= RNG->DR;
            break;
        }
    }
    RNG->CR = control;
    if (!enabled) __HAL_RCC_RNG_CLK_DISABLE();
#endif
#endif
    seed ^= seed >> 16;
    return (uint16_t)seed ? (uint16_t)seed : 1;
}

uint16_t df3EpochNext(void)
{
#ifdef SITL
    static uint16_t saved;
    uint16_t epoch = saved;
#else
    uint16_t epoch = persistentObjectRead(PERSISTENT_OBJECT_DF3_EPOCH);
#endif
    if (!epoch) epoch = bootSeed();
    else if (++epoch == 0) epoch = 1;
#ifdef SITL
    saved = epoch;
#else
    persistentObjectWrite(PERSISTENT_OBJECT_DF3_EPOCH, epoch);
#endif
    return epoch;
}
#endif
