#include "flight/droneforge_policy_runtime.h"
#include "flight/droneforge_policy_weights.h"

#include <math.h>
#include <string.h>

#define DRONEFORGE_POLICY_HIDDEN_DIM 16
#define DRONEFORGE_POLICY_GRU_GATE_DIM (3 * DRONEFORGE_POLICY_HIDDEN_DIM)
#define DRONEFORGE_POLICY_GRU_RESET_OFFSET 0
#define DRONEFORGE_POLICY_GRU_UPDATE_OFFSET DRONEFORGE_POLICY_HIDDEN_DIM
#define DRONEFORGE_POLICY_GRU_CANDIDATE_OFFSET (2 * DRONEFORGE_POLICY_HIDDEN_DIM)

typedef struct {
    float hidden[DRONEFORGE_POLICY_HIDDEN_DIM];
} droneforgePolicyRuntimeState_t;

static droneforgePolicyRuntimeState_t runtimeState;

static float droneforgePolicySigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

static float droneforgePolicyDot(const float *lhs, const float *rhs, int count)
{
    float value = 0.0f;

    for (int i = 0; i < count; i++) {
        value += lhs[i] * rhs[i];
    }

    return value;
}

bool droneforgePolicyRuntimeInit(void)
{
    droneforgePolicyRuntimeReset();
    return true;
}

void droneforgePolicyRuntimeReset(void)
{
    memset(&runtimeState, 0, sizeof(runtimeState));
}

bool droneforgePolicyRuntimeStep(uint32_t nowUs,
                                 const droneforgePolicyObservation_t *observation,
                                 float action[DRONEFORGE_POLICY_ACTION_DIM])
{
    float hiddenInput[DRONEFORGE_POLICY_HIDDEN_DIM];
    float nextHidden[DRONEFORGE_POLICY_HIDDEN_DIM];

    (void)nowUs;

    for (int i = 0; i < DRONEFORGE_POLICY_HIDDEN_DIM; i++) {
        float value = droneforgePolicyDense0Biases[i]
            + droneforgePolicyDot(droneforgePolicyDense0Weights[i],
                                  observation->values,
                                  DRONEFORGE_POLICY_OBSERVATION_DIM);
        hiddenInput[i] = value > 0.0f ? value : 0.0f;
    }

    for (int i = 0; i < DRONEFORGE_POLICY_HIDDEN_DIM; i++) {
        const int resetIndex = DRONEFORGE_POLICY_GRU_RESET_OFFSET + i;
        const int updateIndex = DRONEFORGE_POLICY_GRU_UPDATE_OFFSET + i;
        const int candidateIndex = DRONEFORGE_POLICY_GRU_CANDIDATE_OFFSET + i;

        const float resetGate = droneforgePolicySigmoid(
            droneforgePolicyGruBiasesHidden[resetIndex]
            + droneforgePolicyDot(droneforgePolicyGruWeightsHidden[resetIndex],
                                  runtimeState.hidden,
                                  DRONEFORGE_POLICY_HIDDEN_DIM)
            + droneforgePolicyGruBiasesInput[resetIndex]
            + droneforgePolicyDot(droneforgePolicyGruWeightsInput[resetIndex],
                                  hiddenInput,
                                  DRONEFORGE_POLICY_HIDDEN_DIM));

        const float updateGate = droneforgePolicySigmoid(
            droneforgePolicyGruBiasesHidden[updateIndex]
            + droneforgePolicyDot(droneforgePolicyGruWeightsHidden[updateIndex],
                                  runtimeState.hidden,
                                  DRONEFORGE_POLICY_HIDDEN_DIM)
            + droneforgePolicyGruBiasesInput[updateIndex]
            + droneforgePolicyDot(droneforgePolicyGruWeightsInput[updateIndex],
                                  hiddenInput,
                                  DRONEFORGE_POLICY_HIDDEN_DIM));

        const float candidate = tanhf(
            droneforgePolicyGruBiasesInput[candidateIndex]
            + droneforgePolicyDot(droneforgePolicyGruWeightsInput[candidateIndex],
                                  hiddenInput,
                                  DRONEFORGE_POLICY_HIDDEN_DIM)
            + resetGate * (
                droneforgePolicyGruBiasesHidden[candidateIndex]
                + droneforgePolicyDot(droneforgePolicyGruWeightsHidden[candidateIndex],
                                      runtimeState.hidden,
                                      DRONEFORGE_POLICY_HIDDEN_DIM)));

        nextHidden[i] = (1.0f - updateGate) * candidate + updateGate * runtimeState.hidden[i];
    }

    for (int i = 0; i < DRONEFORGE_POLICY_ACTION_DIM; i++) {
        action[i] = droneforgePolicyDense2Biases[i]
            + droneforgePolicyDot(droneforgePolicyDense2Weights[i],
                                  nextHidden,
                                  DRONEFORGE_POLICY_HIDDEN_DIM);
    }

    memcpy(runtimeState.hidden, nextHidden, sizeof(runtimeState.hidden));
    return true;
}
