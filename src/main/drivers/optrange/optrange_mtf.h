#pragma once

#include "common/time.h"
#include "drivers/optrange/optrange.h"

#define OPTRANGE_MTF_TASK_PERIOD_MS 20

bool mtf02Detect(optrangeDev_t *dev);