#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define KINEMATIC_STATE_DIM 10
#define KINEMATIC_ERROR_DIM 10
#define KINEMATIC_MAIN_ERROR_DIM 10
#define KINEMATIC_CONTROL_DIM 7
#define KINEMATIC_HAS_CONTROL 1
#define KINEMATIC_HAS_SYMBOLIC_Q 1

void kinematic_normalize_state(float *state);
void kinematic_err_fun(float *nom_x, float *delta_x, float *out);
void kinematic_inv_err_fun(float *nom_x, float *true_x, float *out);
void kinematic_H_mod_fun(float *state, float *out);
void kinematic_f_fun(float *state, float *control, float dt, float *out);
void kinematic_F_fun(float *state, float *control, float dt, float *out);
void kinematic_Q_fun(float *state, float *control, float dt, float *out);
#define KINEMATIC_OBS_DIM_2 1
#define KINEMATIC_EXTRA_DIM_2 0
void kinematic_h_2(float *state, float *extra_args, float *out);
void kinematic_H_2(float *state, float *extra_args, float *out);
void kinematic_update_2(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea);
#define KINEMATIC_OBS_DIM_3 1
#define KINEMATIC_EXTRA_DIM_3 0
void kinematic_h_3(float *state, float *extra_args, float *out);
void kinematic_H_3(float *state, float *extra_args, float *out);
void kinematic_update_3(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea);
#define KINEMATIC_OBS_DIM_4 2
#define KINEMATIC_EXTRA_DIM_4 4
void kinematic_h_4(float *state, float *extra_args, float *out);
void kinematic_H_4(float *state, float *extra_args, float *out);
void kinematic_update_4(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea);
void kinematic_predict(float *in_x, float *in_P, float *in_u, float dt);

#ifdef __cplusplus
}
#endif
