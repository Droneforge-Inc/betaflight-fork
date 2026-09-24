#pragma once
#include <stdint.h>
#define USE_DF3
typedef int IRQn_Type;
#define USE_HAL_DRIVER
#define STM32G4
#define U_ID_0 0u
#define U_ID_1 0u
#define U_ID_2 0u
typedef struct { volatile uint32_t CR, SR, DR; } RNG_TypeDef;
typedef struct { volatile uint32_t AHB2ENR, CSR; } RCC_TypeDef;
typedef struct { void *Instance; } RTC_HandleTypeDef;
extern RNG_TypeDef fakeRng;
extern RCC_TypeDef fakeRcc;
#define RNG (&fakeRng)
#define RCC (&fakeRcc)
#define RTC ((void *)1)
#define RCC_AHB2ENR_RNGEN 1u
#define RCC_CSR_SFTRSTF 1u
#define RNG_CR_RNGEN 4u
#define RNG_SR_DRDY 1u
#define RNG_SR_CECS 2u
#define RNG_SR_SECS 4u
#define __HAL_RCC_RNG_CLK_ENABLE() (RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN)
#define __HAL_RCC_RNG_CLK_DISABLE() (RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN)
#define __HAL_RCC_PWR_CLK_ENABLE() ((void)0)
#define HAL_PWR_EnableBkUpAccess() ((void)0)
#define __HAL_RCC_RTCAPB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_RTC_ENABLE() ((void)0)
#define __HAL_RTC_WRITEPROTECTION_ENABLE(h) ((void)0)
#define __HAL_RTC_WRITEPROTECTION_DISABLE(h) ((void)0)
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *, unsigned);
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *, unsigned, uint32_t);
