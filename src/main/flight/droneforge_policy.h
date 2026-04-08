#pragma once

#include <stdbool.h>

#include "common/time.h"

#ifdef __cplusplus
extern "C" {
#endif

void droneforgePolicyInit(void);
void droneforgePolicyReset(void);
bool droneforgePolicyRun(timeUs_t currentTimeUs);

#ifdef __cplusplus
}
#endif
