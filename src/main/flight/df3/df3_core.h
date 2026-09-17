/* Droneforge DF3: bounded-memory port of the Rednose dfEsekf model. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { DF3_NX = 19, DF3_NE = 18 };
enum { DF3_P = 0, DF3_V = 3, DF3_A = 6, DF3_Q = 9, DF3_BA = 13, DF3_BG = 16 };
enum { DF3_EP = 0, DF3_EV = 3, DF3_EA = 6, DF3_ET = 9, DF3_EBA = 12, DF3_EBG = 15 };
typedef enum {
    DF3_OK = 0,
    DF3_REJECTED = 1,
    DF3_INVALID_ARGUMENT = -1,
    DF3_INVALID_QUATERNION = -2,
    DF3_NONFINITE = -3,
    DF3_NOT_POSITIVE = -4
} df3Status_e;
typedef enum {
    DF3_POSITION = 1,
    DF3_VELOCITY = 2,
    DF3_ACCELERATION = 3,
    DF3_BODY_ACCELEROMETER = 4,
    DF3_ATTITUDE = 5
} df3Observation_e;

/* Local/body FRD; Hamilton scalar-first q_LB; right attitude perturbation.
 * x = [p, v, physical a, q, accel bias, gyro bias]. P is row-major.
 * A stationary level body accelerometer measures [0,0,-9.80665]. */
typedef struct {
    float x[DF3_NX];
    float P[DF3_NE * DF3_NE];
} df3Core_t;

/* Caller-owned reusable workspace: no heap, shared globals, or large stack
 * matrices. Do not share one workspace between concurrent core operations. */
typedef struct {
    float F[DF3_NE * DF3_NE];
    float tmp[DF3_NE * DF3_NE];
    float next[DF3_NE * DF3_NE];
} df3Workspace_t;

typedef struct {
    float nis;
    float residualNorm;
    float correctionNorm;
} df3Diagnostics_t;

void df3CoreReset(df3Core_t *core);
bool df3CoreInitialize(df3Core_t *core, const float p[3], const float v[3], const float a[3], const float q[4]);
bool df3CoreHealthy(const df3Core_t *core);
df3Status_e df3CorePredict(df3Core_t *core, df3Workspace_t *work, const float gyro[3], float dt);
df3Status_e df3CoreUpdate(df3Core_t *core, df3Workspace_t *work, df3Observation_e kind, const float z[3],
                          const float R[9], const float measuredQ[4], float nisThreshold,
                          df3Diagnostics_t *diagnostics);

/* Exposed pure model functions support parity and finite-difference checks. */
void df3ModelPredict(const float x[DF3_NX], const float gyro[3], float dt, float next[DF3_NX], float F[DF3_NE * DF3_NE],
                     float Q[DF3_NE * DF3_NE]);
bool df3ModelObserve(const float x[DF3_NX], df3Observation_e kind, const float measuredQ[4], float h[3],
                     float H[3 * DF3_NE]);
void df3QuaternionMultiply(const float a[4], const float b[4], float result[4]);
void df3QuaternionRotate(const float q[4], const float v[3], float result[3]);
void df3QuaternionMatrix(const float q[4], float R[9]);
bool df3ProjectNominal(float x[DF3_NX], const float gyro[3], float dt);

/* Linearized sensor observation, using the same Joseph update/injection as
 * the original model. h and H must describe the supplied current core state. */
df3Status_e df3CoreUpdateObservation(df3Core_t *core, df3Workspace_t *work, const float z[3], const float R[9],
                                     const float h[3], const float H[3 * DF3_NE], float nisThreshold,
                                     df3Diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif
