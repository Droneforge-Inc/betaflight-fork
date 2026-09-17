#pragma once
#include "df3_protocol.h"
#include "df3_estimator.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Version/flags, navigation epoch, sequence, source ms, p(cm), v(mm/s),
 * a(mm/s2), FRD roll/pitch/yaw(mrad). Every multibyte value is big endian. */
void df3StateEncode(const df3Estimate_t *state, uint16_t epoch, uint16_t sequence, bool armed, bool assist,
                    uint8_t payload[DF3_STATE_BYTES]);
#ifdef __cplusplus
}
#endif
