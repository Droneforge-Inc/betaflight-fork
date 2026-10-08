#include "ap_lifecycle.h"
#ifdef USE_AP_AUTONOMY

#include <math.h>
#include <string.h>

static bool stateValid(const df3Estimate_t *estimate)
{
    if (!estimate || !estimate->valid) {
        return false;
    }
    for (unsigned i = 0; i < DF3_NX; ++i) {
        if (!isfinite(estimate->x[i])) {
            return false;
        }
    }
    return true;
}

void apLifecycleFault(apLifecycle_t *state, apLifecycleOutput_t *output)
{
    state->fault = true;
    *output = (apLifecycleOutput_t){.mode = DF3_CONTROL_FAULT, .authority = true};
}

void apLifecycleStep(apLifecycle_t *state, const apLifecycleInput_t *input, apLifecycleOutput_t *output)
{
    *output = (apLifecycleOutput_t){0};
    if (!input->selected || !input->armed || !input->permit || input->nativeOverride) {
        memset(state, 0, sizeof(*state));
        return;
    }
    output->mode = DF3_CONTROL_WAIT;
    const uint64_t now = input->nowUs;
    const float dt = state->lastUs && now > state->lastUs ?
        (now - state->lastUs) * 1e-6f : DF3_CONTROL_INITIAL_DT_S;
    if (state->lastUs && (now <= state->lastUs || now - state->lastUs > DF3_CONTROL_MAX_GAP_US)) {
        state->fault = true;
    }
    state->lastUs = now;
    if (!input->imuFresh || !stateValid(input->estimate)) {
        if (!state->engaged) {
            return;
        }
        state->fault = true;
    }
    if (state->fault) {
        apLifecycleFault(state, output);
        return;
    }
    const df3Reference_t *reference = input->reference;
    const bool referenceValid = reference && reference->active;
    if (!state->engaged) {
        if (!referenceValid || !input->verticalFresh || !input->lateralFresh ||
            !input->estimate->verticalReferenceValid) {
            return;
        }
        state->engaged = true;
    }
    output->authority = true;
    output->lateralControl = input->lateralFresh;
    if (referenceValid && input->verticalFresh && input->lateralFresh) {
        output->target = *reference;
        state->lostReferenceUs = 0;
        output->mode = DF3_CONTROL_TRACK;
    } else {
        if (!state->lostReferenceUs) {
            state->lostReferenceUs = now;
            state->fallback = (df3Reference_t){.active = true};
            memcpy(state->fallback.position, input->estimate->x + DF3_P, sizeof(state->fallback.position));
            const float *q = input->estimate->x + DF3_Q;
            state->fallback.yaw = atan2f(2 * (q[0] * q[3] + q[1] * q[2]),
                                        1 - 2 * (q[2] * q[2] + q[3] * q[3]));
        }
        output->target = state->fallback;
        output->mode = DF3_CONTROL_HOLD;
        if (now - state->lostReferenceUs > DF3_CONTROL_REFERENCE_HOLD_US || !input->lateralFresh) {
            output->mode = DF3_CONTROL_LAND;
            output->target.velocity[2] = DF3_CONTROL_LAND_SPEED_M_S;
            state->fallback.position[2] += DF3_CONTROL_LAND_SPEED_M_S * dt;
            output->target.position[2] = state->fallback.position[2];
        }
        if (!input->verticalFresh) {
            apLifecycleFault(state, output);
            return;
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(output->target.position[i]) || !isfinite(output->target.velocity[i]) ||
            !isfinite(output->target.acceleration[i])) {
            apLifecycleFault(state, output);
            return;
        }
    }
    if (!isfinite(output->target.yaw)) {
        apLifecycleFault(state, output);
    }
}
#endif
