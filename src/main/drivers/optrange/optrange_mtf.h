#pragma once

#include "common/time.h"
#include "drivers/optrange/optrange.h"

#define OPTRANGE_MTF_TASK_PERIOD_MS 20

bool mtf02Detect(optrangeDev_t *dev);

#ifdef SITL
// Diagnostic observations, not health or validation decisions.
uint32_t mtfRangefinderFrameSequence(void);
uint32_t mtfRangefinderFrameAgeMs(void);
void mtfGetLatestRawFlow(int16_t *velX, int16_t *velY);
#endif
