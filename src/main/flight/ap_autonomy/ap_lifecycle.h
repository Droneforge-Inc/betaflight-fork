#pragma once

#include "flight/df3/df3_control.h"

// Flight-policy state only. The AP backend owns its controller integrators.
typedef struct {
    uint64_t lastUs, lostReferenceUs;
    bool engaged, fault;
    df3Reference_t fallback;
} apLifecycle_t;

typedef struct {
    uint64_t nowUs;
    bool armed, selected, permit, nativeOverride;
    bool imuFresh, verticalFresh, lateralFresh;
    const df3Estimate_t *estimate;
    const df3Reference_t *reference;
} apLifecycleInput_t;

typedef struct {
    df3ControlMode_e mode;
    bool authority, lateralControl;
    df3Reference_t target;
} apLifecycleOutput_t;

void apLifecycleStep(apLifecycle_t *state, const apLifecycleInput_t *input, apLifecycleOutput_t *output);
void apLifecycleFault(apLifecycle_t *state, apLifecycleOutput_t *output);
