#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRONEFORGE_POLICY_OBSERVATION_DIM 22
#define DRONEFORGE_POLICY_ACTION_DIM 4

typedef struct {
    float values[DRONEFORGE_POLICY_OBSERVATION_DIM];
} droneforgePolicyObservation_t;

bool droneforgePolicyRuntimeInit(void);
void droneforgePolicyRuntimeReset(void);
bool droneforgePolicyRuntimeStep(uint32_t nowUs,
                                 const droneforgePolicyObservation_t *observation,
                                 float action[DRONEFORGE_POLICY_ACTION_DIM]);

#ifdef __cplusplus
}
#endif
