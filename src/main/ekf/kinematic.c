#include "kinematic.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define DIM 10
#define EDIM 10
#define MEDIM 10
#define MAX_ZDIM 2
#define Q_SYM_IS_DISCRETE 1
typedef void (*ObsFun)(float *, float *, float *);

static const float MAHA_THRESH_2 = 3.8414588206941227f;
static const float MAHA_THRESH_3 = 3.8414588206941227f;
static const float MAHA_THRESH_4 = 5.9914645471079808f;


/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
static void err_fun(float *nom_x, float *delta_x, float *out_8400715136570573408) {
   out_8400715136570573408[0] = delta_x[0] + nom_x[0];
   out_8400715136570573408[1] = delta_x[1] + nom_x[1];
   out_8400715136570573408[2] = delta_x[2] + nom_x[2];
   out_8400715136570573408[3] = delta_x[3] + nom_x[3];
   out_8400715136570573408[4] = delta_x[4] + nom_x[4];
   out_8400715136570573408[5] = delta_x[5] + nom_x[5];
   out_8400715136570573408[6] = delta_x[6] + nom_x[6];
   out_8400715136570573408[7] = delta_x[7] + nom_x[7];
   out_8400715136570573408[8] = delta_x[8] + nom_x[8];
   out_8400715136570573408[9] = delta_x[9] + nom_x[9];
}
static void inv_err_fun(float *nom_x, float *true_x, float *out_5838504077766560278) {
   out_5838504077766560278[0] = -nom_x[0] + true_x[0];
   out_5838504077766560278[1] = -nom_x[1] + true_x[1];
   out_5838504077766560278[2] = -nom_x[2] + true_x[2];
   out_5838504077766560278[3] = -nom_x[3] + true_x[3];
   out_5838504077766560278[4] = -nom_x[4] + true_x[4];
   out_5838504077766560278[5] = -nom_x[5] + true_x[5];
   out_5838504077766560278[6] = -nom_x[6] + true_x[6];
   out_5838504077766560278[7] = -nom_x[7] + true_x[7];
   out_5838504077766560278[8] = -nom_x[8] + true_x[8];
   out_5838504077766560278[9] = -nom_x[9] + true_x[9];
}
static void H_mod_fun(float *state, float *out_5879640092950765585) {
  (void)state;
   out_5879640092950765585[0] = 1.0;
   out_5879640092950765585[1] = 0.0;
   out_5879640092950765585[2] = 0.0;
   out_5879640092950765585[3] = 0.0;
   out_5879640092950765585[4] = 0.0;
   out_5879640092950765585[5] = 0.0;
   out_5879640092950765585[6] = 0.0;
   out_5879640092950765585[7] = 0.0;
   out_5879640092950765585[8] = 0.0;
   out_5879640092950765585[9] = 0.0;
   out_5879640092950765585[10] = 0.0;
   out_5879640092950765585[11] = 1.0;
   out_5879640092950765585[12] = 0.0;
   out_5879640092950765585[13] = 0.0;
   out_5879640092950765585[14] = 0.0;
   out_5879640092950765585[15] = 0.0;
   out_5879640092950765585[16] = 0.0;
   out_5879640092950765585[17] = 0.0;
   out_5879640092950765585[18] = 0.0;
   out_5879640092950765585[19] = 0.0;
   out_5879640092950765585[20] = 0.0;
   out_5879640092950765585[21] = 0.0;
   out_5879640092950765585[22] = 1.0;
   out_5879640092950765585[23] = 0.0;
   out_5879640092950765585[24] = 0.0;
   out_5879640092950765585[25] = 0.0;
   out_5879640092950765585[26] = 0.0;
   out_5879640092950765585[27] = 0.0;
   out_5879640092950765585[28] = 0.0;
   out_5879640092950765585[29] = 0.0;
   out_5879640092950765585[30] = 0.0;
   out_5879640092950765585[31] = 0.0;
   out_5879640092950765585[32] = 0.0;
   out_5879640092950765585[33] = 1.0;
   out_5879640092950765585[34] = 0.0;
   out_5879640092950765585[35] = 0.0;
   out_5879640092950765585[36] = 0.0;
   out_5879640092950765585[37] = 0.0;
   out_5879640092950765585[38] = 0.0;
   out_5879640092950765585[39] = 0.0;
   out_5879640092950765585[40] = 0.0;
   out_5879640092950765585[41] = 0.0;
   out_5879640092950765585[42] = 0.0;
   out_5879640092950765585[43] = 0.0;
   out_5879640092950765585[44] = 1.0;
   out_5879640092950765585[45] = 0.0;
   out_5879640092950765585[46] = 0.0;
   out_5879640092950765585[47] = 0.0;
   out_5879640092950765585[48] = 0.0;
   out_5879640092950765585[49] = 0.0;
   out_5879640092950765585[50] = 0.0;
   out_5879640092950765585[51] = 0.0;
   out_5879640092950765585[52] = 0.0;
   out_5879640092950765585[53] = 0.0;
   out_5879640092950765585[54] = 0.0;
   out_5879640092950765585[55] = 1.0;
   out_5879640092950765585[56] = 0.0;
   out_5879640092950765585[57] = 0.0;
   out_5879640092950765585[58] = 0.0;
   out_5879640092950765585[59] = 0.0;
   out_5879640092950765585[60] = 0.0;
   out_5879640092950765585[61] = 0.0;
   out_5879640092950765585[62] = 0.0;
   out_5879640092950765585[63] = 0.0;
   out_5879640092950765585[64] = 0.0;
   out_5879640092950765585[65] = 0.0;
   out_5879640092950765585[66] = 1.0;
   out_5879640092950765585[67] = 0.0;
   out_5879640092950765585[68] = 0.0;
   out_5879640092950765585[69] = 0.0;
   out_5879640092950765585[70] = 0.0;
   out_5879640092950765585[71] = 0.0;
   out_5879640092950765585[72] = 0.0;
   out_5879640092950765585[73] = 0.0;
   out_5879640092950765585[74] = 0.0;
   out_5879640092950765585[75] = 0.0;
   out_5879640092950765585[76] = 0.0;
   out_5879640092950765585[77] = 1.0;
   out_5879640092950765585[78] = 0.0;
   out_5879640092950765585[79] = 0.0;
   out_5879640092950765585[80] = 0.0;
   out_5879640092950765585[81] = 0.0;
   out_5879640092950765585[82] = 0.0;
   out_5879640092950765585[83] = 0.0;
   out_5879640092950765585[84] = 0.0;
   out_5879640092950765585[85] = 0.0;
   out_5879640092950765585[86] = 0.0;
   out_5879640092950765585[87] = 0.0;
   out_5879640092950765585[88] = 1.0;
   out_5879640092950765585[89] = 0.0;
   out_5879640092950765585[90] = 0.0;
   out_5879640092950765585[91] = 0.0;
   out_5879640092950765585[92] = 0.0;
   out_5879640092950765585[93] = 0.0;
   out_5879640092950765585[94] = 0.0;
   out_5879640092950765585[95] = 0.0;
   out_5879640092950765585[96] = 0.0;
   out_5879640092950765585[97] = 0.0;
   out_5879640092950765585[98] = 0.0;
   out_5879640092950765585[99] = 1.0;
}
static void f_fun(float *state, float *u, float dt, float *out_3330065704444319255) {
   out_3330065704444319255[0] = (1.0/2.0)*powf(dt, 2)*((2*u[3]*u[5] + 2*u[4]*u[6])*(-state[9] + u[2]) + (-2*u[3]*u[6] + 2*u[4]*u[5])*(-state[8] + u[1]) + (-state[7] + u[0])*(powf(u[3], 2) + powf(u[4], 2) - powf(u[5], 2) - powf(u[6], 2))) + dt*state[3] + state[0];
   out_3330065704444319255[1] = (1.0/2.0)*powf(dt, 2)*((-2*u[3]*u[4] + 2*u[5]*u[6])*(-state[9] + u[2]) + (2*u[3]*u[6] + 2*u[4]*u[5])*(-state[7] + u[0]) + (-state[8] + u[1])*(powf(u[3], 2) - powf(u[4], 2) + powf(u[5], 2) - powf(u[6], 2))) + dt*state[4] + state[1];
   out_3330065704444319255[2] = (1.0/2.0)*powf(dt, 2)*((2*u[3]*u[4] + 2*u[5]*u[6])*(-state[8] + u[1]) + (-2*u[3]*u[5] + 2*u[4]*u[6])*(-state[7] + u[0]) + (-state[9] + u[2])*(powf(u[3], 2) - powf(u[4], 2) - powf(u[5], 2) + powf(u[6], 2)) - 9.8066499999999994) + dt*state[5] + state[2];
   out_3330065704444319255[3] = dt*((2*u[3]*u[5] + 2*u[4]*u[6])*(-state[9] + u[2]) + (-2*u[3]*u[6] + 2*u[4]*u[5])*(-state[8] + u[1]) + (-state[7] + u[0])*(powf(u[3], 2) + powf(u[4], 2) - powf(u[5], 2) - powf(u[6], 2))) + state[3];
   out_3330065704444319255[4] = dt*((-2*u[3]*u[4] + 2*u[5]*u[6])*(-state[9] + u[2]) + (2*u[3]*u[6] + 2*u[4]*u[5])*(-state[7] + u[0]) + (-state[8] + u[1])*(powf(u[3], 2) - powf(u[4], 2) + powf(u[5], 2) - powf(u[6], 2))) + state[4];
   out_3330065704444319255[5] = dt*((2*u[3]*u[4] + 2*u[5]*u[6])*(-state[8] + u[1]) + (-2*u[3]*u[5] + 2*u[4]*u[6])*(-state[7] + u[0]) + (-state[9] + u[2])*(powf(u[3], 2) - powf(u[4], 2) - powf(u[5], 2) + powf(u[6], 2)) - 9.8066499999999994) + state[5];
   out_3330065704444319255[6] = state[6];
   out_3330065704444319255[7] = state[7];
   out_3330065704444319255[8] = state[8];
   out_3330065704444319255[9] = state[9];
}
static void F_fun(float *state, float *u, float dt, float *out_3624662214306482855) {
  (void)state;
   out_3624662214306482855[0] = 1;
   out_3624662214306482855[1] = 0;
   out_3624662214306482855[2] = 0;
   out_3624662214306482855[3] = dt;
   out_3624662214306482855[4] = 0;
   out_3624662214306482855[5] = 0;
   out_3624662214306482855[6] = 0;
   out_3624662214306482855[7] = (1.0/2.0)*powf(dt, 2)*(-powf(u[3], 2) - powf(u[4], 2) + powf(u[5], 2) + powf(u[6], 2));
   out_3624662214306482855[8] = (1.0/2.0)*powf(dt, 2)*(2*u[3]*u[6] - 2*u[4]*u[5]);
   out_3624662214306482855[9] = (1.0/2.0)*powf(dt, 2)*(-2*u[3]*u[5] - 2*u[4]*u[6]);
   out_3624662214306482855[10] = 0;
   out_3624662214306482855[11] = 1;
   out_3624662214306482855[12] = 0;
   out_3624662214306482855[13] = 0;
   out_3624662214306482855[14] = dt;
   out_3624662214306482855[15] = 0;
   out_3624662214306482855[16] = 0;
   out_3624662214306482855[17] = (1.0/2.0)*powf(dt, 2)*(-2*u[3]*u[6] - 2*u[4]*u[5]);
   out_3624662214306482855[18] = (1.0/2.0)*powf(dt, 2)*(-powf(u[3], 2) + powf(u[4], 2) - powf(u[5], 2) + powf(u[6], 2));
   out_3624662214306482855[19] = (1.0/2.0)*powf(dt, 2)*(2*u[3]*u[4] - 2*u[5]*u[6]);
   out_3624662214306482855[20] = 0;
   out_3624662214306482855[21] = 0;
   out_3624662214306482855[22] = 1;
   out_3624662214306482855[23] = 0;
   out_3624662214306482855[24] = 0;
   out_3624662214306482855[25] = dt;
   out_3624662214306482855[26] = 0;
   out_3624662214306482855[27] = (1.0/2.0)*powf(dt, 2)*(2*u[3]*u[5] - 2*u[4]*u[6]);
   out_3624662214306482855[28] = (1.0/2.0)*powf(dt, 2)*(-2*u[3]*u[4] - 2*u[5]*u[6]);
   out_3624662214306482855[29] = (1.0/2.0)*powf(dt, 2)*(-powf(u[3], 2) + powf(u[4], 2) + powf(u[5], 2) - powf(u[6], 2));
   out_3624662214306482855[30] = 0;
   out_3624662214306482855[31] = 0;
   out_3624662214306482855[32] = 0;
   out_3624662214306482855[33] = 1;
   out_3624662214306482855[34] = 0;
   out_3624662214306482855[35] = 0;
   out_3624662214306482855[36] = 0;
   out_3624662214306482855[37] = dt*(-powf(u[3], 2) - powf(u[4], 2) + powf(u[5], 2) + powf(u[6], 2));
   out_3624662214306482855[38] = dt*(2*u[3]*u[6] - 2*u[4]*u[5]);
   out_3624662214306482855[39] = dt*(-2*u[3]*u[5] - 2*u[4]*u[6]);
   out_3624662214306482855[40] = 0;
   out_3624662214306482855[41] = 0;
   out_3624662214306482855[42] = 0;
   out_3624662214306482855[43] = 0;
   out_3624662214306482855[44] = 1;
   out_3624662214306482855[45] = 0;
   out_3624662214306482855[46] = 0;
   out_3624662214306482855[47] = dt*(-2*u[3]*u[6] - 2*u[4]*u[5]);
   out_3624662214306482855[48] = dt*(-powf(u[3], 2) + powf(u[4], 2) - powf(u[5], 2) + powf(u[6], 2));
   out_3624662214306482855[49] = dt*(2*u[3]*u[4] - 2*u[5]*u[6]);
   out_3624662214306482855[50] = 0;
   out_3624662214306482855[51] = 0;
   out_3624662214306482855[52] = 0;
   out_3624662214306482855[53] = 0;
   out_3624662214306482855[54] = 0;
   out_3624662214306482855[55] = 1;
   out_3624662214306482855[56] = 0;
   out_3624662214306482855[57] = dt*(2*u[3]*u[5] - 2*u[4]*u[6]);
   out_3624662214306482855[58] = dt*(-2*u[3]*u[4] - 2*u[5]*u[6]);
   out_3624662214306482855[59] = dt*(-powf(u[3], 2) + powf(u[4], 2) + powf(u[5], 2) - powf(u[6], 2));
   out_3624662214306482855[60] = 0;
   out_3624662214306482855[61] = 0;
   out_3624662214306482855[62] = 0;
   out_3624662214306482855[63] = 0;
   out_3624662214306482855[64] = 0;
   out_3624662214306482855[65] = 0;
   out_3624662214306482855[66] = 1;
   out_3624662214306482855[67] = 0;
   out_3624662214306482855[68] = 0;
   out_3624662214306482855[69] = 0;
   out_3624662214306482855[70] = 0;
   out_3624662214306482855[71] = 0;
   out_3624662214306482855[72] = 0;
   out_3624662214306482855[73] = 0;
   out_3624662214306482855[74] = 0;
   out_3624662214306482855[75] = 0;
   out_3624662214306482855[76] = 0;
   out_3624662214306482855[77] = 1;
   out_3624662214306482855[78] = 0;
   out_3624662214306482855[79] = 0;
   out_3624662214306482855[80] = 0;
   out_3624662214306482855[81] = 0;
   out_3624662214306482855[82] = 0;
   out_3624662214306482855[83] = 0;
   out_3624662214306482855[84] = 0;
   out_3624662214306482855[85] = 0;
   out_3624662214306482855[86] = 0;
   out_3624662214306482855[87] = 0;
   out_3624662214306482855[88] = 1;
   out_3624662214306482855[89] = 0;
   out_3624662214306482855[90] = 0;
   out_3624662214306482855[91] = 0;
   out_3624662214306482855[92] = 0;
   out_3624662214306482855[93] = 0;
   out_3624662214306482855[94] = 0;
   out_3624662214306482855[95] = 0;
   out_3624662214306482855[96] = 0;
   out_3624662214306482855[97] = 0;
   out_3624662214306482855[98] = 0;
   out_3624662214306482855[99] = 1;
}
static void Q_fun(float *state, float *u, float dt, float *out_8645926798053728310) {
  (void)state;
  (void)u;
   out_8645926798053728310[0] = 0.25*powf(dt, 4);
   out_8645926798053728310[1] = 0;
   out_8645926798053728310[2] = 0;
   out_8645926798053728310[3] = 0.5*powf(dt, 3);
   out_8645926798053728310[4] = 0;
   out_8645926798053728310[5] = 0;
   out_8645926798053728310[6] = 0;
   out_8645926798053728310[7] = 0;
   out_8645926798053728310[8] = 0;
   out_8645926798053728310[9] = 0;
   out_8645926798053728310[10] = 0;
   out_8645926798053728310[11] = 0.25*powf(dt, 4);
   out_8645926798053728310[12] = 0;
   out_8645926798053728310[13] = 0;
   out_8645926798053728310[14] = 0.5*powf(dt, 3);
   out_8645926798053728310[15] = 0;
   out_8645926798053728310[16] = 0;
   out_8645926798053728310[17] = 0;
   out_8645926798053728310[18] = 0;
   out_8645926798053728310[19] = 0;
   out_8645926798053728310[20] = 0;
   out_8645926798053728310[21] = 0;
   out_8645926798053728310[22] = 0.25*powf(dt, 4);
   out_8645926798053728310[23] = 0;
   out_8645926798053728310[24] = 0;
   out_8645926798053728310[25] = 0.5*powf(dt, 3);
   out_8645926798053728310[26] = 0;
   out_8645926798053728310[27] = 0;
   out_8645926798053728310[28] = 0;
   out_8645926798053728310[29] = 0;
   out_8645926798053728310[30] = 0.5*powf(dt, 3);
   out_8645926798053728310[31] = 0;
   out_8645926798053728310[32] = 0;
   out_8645926798053728310[33] = 1.0*powf(dt, 2);
   out_8645926798053728310[34] = 0;
   out_8645926798053728310[35] = 0;
   out_8645926798053728310[36] = 0;
   out_8645926798053728310[37] = 0;
   out_8645926798053728310[38] = 0;
   out_8645926798053728310[39] = 0;
   out_8645926798053728310[40] = 0;
   out_8645926798053728310[41] = 0.5*powf(dt, 3);
   out_8645926798053728310[42] = 0;
   out_8645926798053728310[43] = 0;
   out_8645926798053728310[44] = 1.0*powf(dt, 2);
   out_8645926798053728310[45] = 0;
   out_8645926798053728310[46] = 0;
   out_8645926798053728310[47] = 0;
   out_8645926798053728310[48] = 0;
   out_8645926798053728310[49] = 0;
   out_8645926798053728310[50] = 0;
   out_8645926798053728310[51] = 0;
   out_8645926798053728310[52] = 0.5*powf(dt, 3);
   out_8645926798053728310[53] = 0;
   out_8645926798053728310[54] = 0;
   out_8645926798053728310[55] = 1.0*powf(dt, 2);
   out_8645926798053728310[56] = 0;
   out_8645926798053728310[57] = 0;
   out_8645926798053728310[58] = 0;
   out_8645926798053728310[59] = 0;
   out_8645926798053728310[60] = 0;
   out_8645926798053728310[61] = 0;
   out_8645926798053728310[62] = 0;
   out_8645926798053728310[63] = 0;
   out_8645926798053728310[64] = 0;
   out_8645926798053728310[65] = 0;
   out_8645926798053728310[66] = 0.0001*dt;
   out_8645926798053728310[67] = 0;
   out_8645926798053728310[68] = 0;
   out_8645926798053728310[69] = 0;
   out_8645926798053728310[70] = 0;
   out_8645926798053728310[71] = 0;
   out_8645926798053728310[72] = 0;
   out_8645926798053728310[73] = 0;
   out_8645926798053728310[74] = 0;
   out_8645926798053728310[75] = 0;
   out_8645926798053728310[76] = 0;
   out_8645926798053728310[77] = 2.5000000000000001e-5*dt;
   out_8645926798053728310[78] = 0;
   out_8645926798053728310[79] = 0;
   out_8645926798053728310[80] = 0;
   out_8645926798053728310[81] = 0;
   out_8645926798053728310[82] = 0;
   out_8645926798053728310[83] = 0;
   out_8645926798053728310[84] = 0;
   out_8645926798053728310[85] = 0;
   out_8645926798053728310[86] = 0;
   out_8645926798053728310[87] = 0;
   out_8645926798053728310[88] = 2.5000000000000001e-5*dt;
   out_8645926798053728310[89] = 0;
   out_8645926798053728310[90] = 0;
   out_8645926798053728310[91] = 0;
   out_8645926798053728310[92] = 0;
   out_8645926798053728310[93] = 0;
   out_8645926798053728310[94] = 0;
   out_8645926798053728310[95] = 0;
   out_8645926798053728310[96] = 0;
   out_8645926798053728310[97] = 0;
   out_8645926798053728310[98] = 0;
   out_8645926798053728310[99] = 2.5000000000000001e-5*dt;
}
static void h_2(float *state, float *unused, float *out_6293215938335595047) {
  (void)unused;
   out_6293215938335595047[0] = state[2];
}
static void H_2(float *state, float *unused, float *out_4758000223035142537) {
  (void)state;
  (void)unused;
   out_4758000223035142537[0] = 0;
   out_4758000223035142537[1] = 0;
   out_4758000223035142537[2] = 1;
   out_4758000223035142537[3] = 0;
   out_4758000223035142537[4] = 0;
   out_4758000223035142537[5] = 0;
   out_4758000223035142537[6] = 0;
   out_4758000223035142537[7] = 0;
   out_4758000223035142537[8] = 0;
   out_4758000223035142537[9] = 0;
}
static void h_3(float *state, float *unused, float *out_8182581982159968412) {
  (void)unused;
   out_8182581982159968412[0] = state[2] + state[6];
}
static void H_3(float *state, float *unused, float *out_119584145151623248) {
  (void)state;
  (void)unused;
   out_119584145151623248[0] = 0;
   out_119584145151623248[1] = 0;
   out_119584145151623248[2] = 1;
   out_119584145151623248[3] = 0;
   out_119584145151623248[4] = 0;
   out_119584145151623248[5] = 0;
   out_119584145151623248[6] = 1;
   out_119584145151623248[7] = 0;
   out_119584145151623248[8] = 0;
   out_119584145151623248[9] = 0;
}
static void h_4(float *state, float *flow_q, float *out_4944527014070724334) {
   out_4944527014070724334[0] = (-2*flow_q[0]*flow_q[2] + 2*flow_q[1]*flow_q[3])*state[5] + (2*flow_q[0]*flow_q[3] + 2*flow_q[1]*flow_q[2])*state[4] + (powf(flow_q[0], 2) + powf(flow_q[1], 2) - powf(flow_q[2], 2) - powf(flow_q[3], 2))*state[3];
   out_4944527014070724334[1] = (2*flow_q[0]*flow_q[1] + 2*flow_q[2]*flow_q[3])*state[5] + (-2*flow_q[0]*flow_q[3] + 2*flow_q[1]*flow_q[2])*state[3] + (powf(flow_q[0], 2) - powf(flow_q[1], 2) + powf(flow_q[2], 2) - powf(flow_q[3], 2))*state[4];
}
static void H_4(float *state, float *flow_q, float *out_3268541180516938844) {
  (void)state;
   out_3268541180516938844[0] = 0;
   out_3268541180516938844[1] = 0;
   out_3268541180516938844[2] = 0;
   out_3268541180516938844[3] = powf(flow_q[0], 2) + powf(flow_q[1], 2) - powf(flow_q[2], 2) - powf(flow_q[3], 2);
   out_3268541180516938844[4] = 2*flow_q[0]*flow_q[3] + 2*flow_q[1]*flow_q[2];
   out_3268541180516938844[5] = -2*flow_q[0]*flow_q[2] + 2*flow_q[1]*flow_q[3];
   out_3268541180516938844[6] = 0;
   out_3268541180516938844[7] = 0;
   out_3268541180516938844[8] = 0;
   out_3268541180516938844[9] = 0;
   out_3268541180516938844[10] = 0;
   out_3268541180516938844[11] = 0;
   out_3268541180516938844[12] = 0;
   out_3268541180516938844[13] = -2*flow_q[0]*flow_q[3] + 2*flow_q[1]*flow_q[2];
   out_3268541180516938844[14] = powf(flow_q[0], 2) - powf(flow_q[1], 2) + powf(flow_q[2], 2) - powf(flow_q[3], 2);
   out_3268541180516938844[15] = 2*flow_q[0]*flow_q[1] + 2*flow_q[2]*flow_q[3];
   out_3268541180516938844[16] = 0;
   out_3268541180516938844[17] = 0;
   out_3268541180516938844[18] = 0;
   out_3268541180516938844[19] = 0;
}
static void __attribute__((unused)) normalize_quaternion_slice(float *state, int start, int len) {
  float norm_sq = 0.0f;
  for (int i = 0; i < len; ++i) {
    const float value = state[start + i];
    norm_sq += value * value;
  }

  if (norm_sq <= 0.0f) {
    return;
  }

  const float inv_norm = 1.0f / sqrtf(norm_sq);
  for (int i = 0; i < len; ++i) {
    state[start + i] *= inv_norm;
  }
}


static void mat_mul(const float *a, int a_rows, int a_cols, const float *b, int b_cols, float *out) {
  for (int i = 0; i < a_rows; ++i) {
    for (int j = 0; j < b_cols; ++j) {
      float sum = 0.0f;
      for (int k = 0; k < a_cols; ++k) {
        sum += a[i * a_cols + k] * b[k * b_cols + j];
      }
      out[i * b_cols + j] = sum;
    }
  }
}


static void mat_mul_transpose_right(const float *a, int a_rows, int a_cols, const float *b, int b_rows, float *out) {
  for (int i = 0; i < a_rows; ++i) {
    for (int j = 0; j < b_rows; ++j) {
      float sum = 0.0f;
      for (int k = 0; k < a_cols; ++k) {
        sum += a[i * a_cols + k] * b[j * a_cols + k];
      }
      out[i * b_rows + j] = sum;
    }
  }
}


static void mat_mul_transpose_left(const float *a, int a_rows, int a_cols, const float *b, int b_cols, float *out) {
  for (int i = 0; i < a_cols; ++i) {
    for (int j = 0; j < b_cols; ++j) {
      float sum = 0.0f;
      for (int k = 0; k < a_rows; ++k) {
        sum += a[k * a_cols + i] * b[k * b_cols + j];
      }
      out[i * b_cols + j] = sum;
    }
  }
}


static void mat_transpose_vec_mul(const float *a, int rows, int cols, const float *x, float *out) {
  for (int i = 0; i < cols; ++i) {
    float sum = 0.0f;
    for (int j = 0; j < rows; ++j) {
      sum += a[j * cols + i] * x[j];
    }
    out[i] = sum;
  }
}


static float dot_product(const float *a, const float *b, int len) {
  float sum = 0.0f;
  for (int i = 0; i < len; ++i) {
    sum += a[i] * b[i];
  }
  return sum;
}


static int solve_linear_system(int dim, float *a, float *b, int nrhs) {
  const float pivot_eps = 1.0e-9f;

  for (int col = 0; col < dim; ++col) {
    int pivot_row = col;
    float pivot_abs = fabsf(a[col * dim + col]);

    for (int row = col + 1; row < dim; ++row) {
      const float candidate = fabsf(a[row * dim + col]);
      if (candidate > pivot_abs) {
        pivot_abs = candidate;
        pivot_row = row;
      }
    }

    if (pivot_abs < pivot_eps) {
      return 0;
    }

    if (pivot_row != col) {
      for (int j = 0; j < dim; ++j) {
        const float tmp = a[col * dim + j];
        a[col * dim + j] = a[pivot_row * dim + j];
        a[pivot_row * dim + j] = tmp;
      }
      for (int j = 0; j < nrhs; ++j) {
        const float tmp = b[col * nrhs + j];
        b[col * nrhs + j] = b[pivot_row * nrhs + j];
        b[pivot_row * nrhs + j] = tmp;
      }
    }

    const float pivot = a[col * dim + col];
    const float inv_pivot = 1.0f / pivot;
    for (int j = col; j < dim; ++j) {
      a[col * dim + j] *= inv_pivot;
    }
    for (int j = 0; j < nrhs; ++j) {
      b[col * nrhs + j] *= inv_pivot;
    }

    for (int row = 0; row < dim; ++row) {
      if (row == col) {
        continue;
      }

      const float factor = a[row * dim + col];
      if (factor == 0.0f) {
        continue;
      }

      for (int j = col; j < dim; ++j) {
        a[row * dim + j] -= factor * a[col * dim + j];
      }
      for (int j = 0; j < nrhs; ++j) {
        b[row * nrhs + j] -= factor * b[col * nrhs + j];
      }
    }
  }

  return 1;
}


static void predict_covariance(float *in_P, float *in_Q, const float *in_F, float dt) {
#ifdef Q_SYM_IS_DISCRETE
  (void)dt;
#endif
  float p_prev[EDIM * EDIM] = {0};
  float p_next[EDIM * EDIM] = {0};
  memcpy(p_prev, in_P, sizeof(p_prev));
  memcpy(p_next, p_prev, sizeof(p_next));

  for (int row = 0; row < MEDIM; ++row) {
    for (int col = 0; col < MEDIM; ++col) {
      float sum = 0.0f;
      for (int k = 0; k < MEDIM; ++k) {
        for (int l = 0; l < MEDIM; ++l) {
          sum += in_F[row * EDIM + k] * p_prev[k * EDIM + l] * in_F[col * EDIM + l];
        }
      }
      p_next[row * EDIM + col] = sum;
    }
  }

  for (int row = 0; row < MEDIM; ++row) {
    for (int col = MEDIM; col < EDIM; ++col) {
      float sum = 0.0f;
      for (int k = 0; k < MEDIM; ++k) {
        sum += in_F[row * EDIM + k] * p_prev[k * EDIM + col];
      }
      p_next[row * EDIM + col] = sum;
    }
  }

  for (int row = MEDIM; row < EDIM; ++row) {
    for (int col = 0; col < MEDIM; ++col) {
      float sum = 0.0f;
      for (int k = 0; k < MEDIM; ++k) {
        sum += p_prev[row * EDIM + k] * in_F[col * EDIM + k];
      }
      p_next[row * EDIM + col] = sum;
    }
  }

  for (int i = 0; i < EDIM * EDIM; ++i) {
#ifdef Q_SYM_IS_DISCRETE
    p_next[i] += in_Q[i];
#else
    p_next[i] += dt * in_Q[i];
#endif
  }

  memcpy(in_P, p_next, sizeof(p_next));
}


static int update_core(int zdim, float *in_x, float *in_P, ObsFun h_fun, ObsFun H_fun, float *in_z, float *in_R,
                       float *in_ea, float maha_threshold, int do_maha) {
  float h[MAX_ZDIM] = {0};
  float H[MAX_ZDIM * DIM] = {0};
  float H_mod[DIM * EDIM] = {0};
  float H_err[MAX_ZDIM * EDIM] = {0};
  float y[MAX_ZDIM] = {0};
  float R[MAX_ZDIM * MAX_ZDIM] = {0};
  float HP[MAX_ZDIM * EDIM] = {0};
  float S[MAX_ZDIM * MAX_ZDIM] = {0};
  float S_work[MAX_ZDIM * MAX_ZDIM] = {0};
  float KT[MAX_ZDIM * EDIM] = {0};
  float KH[EDIM * EDIM] = {0};
  float I_KH[EDIM * EDIM] = {0};
  float dx[EDIM] = {0};
  float x_new[DIM] = {0};
  float temp[EDIM * EDIM] = {0};
  float p_new[EDIM * EDIM] = {0};
  float RKT[MAX_ZDIM * EDIM] = {0};
  float KRKT[EDIM * EDIM] = {0};
  float maha_rhs[MAX_ZDIM] = {0};

  h_fun(in_x, in_ea, h);
  H_fun(in_x, in_ea, H);
  H_mod_fun(in_x, H_mod);

  for (int i = 0; i < zdim; ++i) {
    y[i] = in_z[i] - h[i];
    for (int j = 0; j < zdim; ++j) {
      R[i * zdim + j] = in_R[i * zdim + j];
    }
  }

  mat_mul(H, zdim, DIM, H_mod, EDIM, H_err);
  mat_mul(H_err, zdim, EDIM, in_P, EDIM, HP);
  mat_mul_transpose_right(HP, zdim, EDIM, H_err, zdim, S);
  for (int i = 0; i < zdim; ++i) {
    for (int j = 0; j < zdim; ++j) {
      S[i * zdim + j] += R[i * zdim + j];
    }
  }

  if (do_maha) {
    memcpy(S_work, S, (size_t)(zdim * zdim) * sizeof(float));
    memcpy(maha_rhs, y, (size_t)zdim * sizeof(float));
    if (!solve_linear_system(zdim, S_work, maha_rhs, 1)) {
      return 0;
    }
    if (dot_product(y, maha_rhs, zdim) > maha_threshold) {
      for (int i = 0; i < zdim * zdim; ++i) {
        R[i] *= 1.0e16f;
      }
      mat_mul_transpose_right(HP, zdim, EDIM, H_err, zdim, S);
      for (int i = 0; i < zdim; ++i) {
        for (int j = 0; j < zdim; ++j) {
          S[i * zdim + j] += R[i * zdim + j];
        }
      }
    }
  }

  memcpy(S_work, S, (size_t)(zdim * zdim) * sizeof(float));
  memcpy(KT, HP, (size_t)(zdim * EDIM) * sizeof(float));
  if (!solve_linear_system(zdim, S_work, KT, EDIM)) {
    return 0;
  }

  mat_mul_transpose_left(KT, zdim, EDIM, H_err, EDIM, KH);
  for (int i = 0; i < EDIM; ++i) {
    for (int j = 0; j < EDIM; ++j) {
      const float identity = (i == j) ? 1.0f : 0.0f;
      I_KH[i * EDIM + j] = identity - KH[i * EDIM + j];
    }
  }

  mat_transpose_vec_mul(KT, zdim, EDIM, y, dx);
  err_fun(in_x, dx, x_new);
  kinematic_normalize_state(x_new);

  mat_mul(I_KH, EDIM, EDIM, in_P, EDIM, temp);
  mat_mul_transpose_right(temp, EDIM, EDIM, I_KH, EDIM, p_new);
  mat_mul(R, zdim, zdim, KT, EDIM, RKT);
  mat_mul_transpose_left(KT, zdim, EDIM, RKT, EDIM, KRKT);
  for (int i = 0; i < EDIM * EDIM; ++i) {
    p_new[i] += KRKT[i];
  }

  memcpy(in_x, x_new, sizeof(x_new));
  memcpy(in_P, p_new, sizeof(p_new));
  memcpy(in_z, y, (size_t)zdim * sizeof(float));
  return 1;
}

void kinematic_normalize_state(float *state) {
  (void)state;
}

void kinematic_err_fun(float *nom_x, float *delta_x, float *out) {
  err_fun(nom_x, delta_x, out);
}

void kinematic_inv_err_fun(float *nom_x, float *true_x, float *out) {
  inv_err_fun(nom_x, true_x, out);
}

void kinematic_H_mod_fun(float *state, float *out) {
  H_mod_fun(state, out);
}

void kinematic_f_fun(float *state, float *control, float dt, float *out) {
  f_fun(state, control, dt, out);
}

void kinematic_F_fun(float *state, float *control, float dt, float *out) {
  F_fun(state, control, dt, out);
}

void kinematic_Q_fun(float *state, float *control, float dt, float *out) {
  Q_fun(state, control, dt, out);
}

void kinematic_h_2(float *state, float *extra_args, float *out) {
  h_2(state, extra_args, out);
}

void kinematic_H_2(float *state, float *extra_args, float *out) {
  H_2(state, extra_args, out);
}

void kinematic_h_3(float *state, float *extra_args, float *out) {
  h_3(state, extra_args, out);
}

void kinematic_H_3(float *state, float *extra_args, float *out) {
  H_3(state, extra_args, out);
}

void kinematic_h_4(float *state, float *extra_args, float *out) {
  h_4(state, extra_args, out);
}

void kinematic_H_4(float *state, float *extra_args, float *out) {
  H_4(state, extra_args, out);
}

void kinematic_predict(float *in_x, float *in_P, float *in_u, float dt) {
  float nx[DIM] = {0};
  float in_F[EDIM * EDIM] = {0};
  float in_Q_dyn[EDIM * EDIM] = {0};
  f_fun(in_x, in_u, dt, nx);
  F_fun(in_x, in_u, dt, in_F);
  Q_fun(in_x, in_u, dt, in_Q_dyn);
  predict_covariance(in_P, in_Q_dyn, in_F, dt);
  memcpy(in_x, nx, DIM * sizeof(float));
  kinematic_normalize_state(in_x);
}

void kinematic_update_2(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea) {
  (void)update_core(1, in_x, in_P, h_2, H_2, in_z, in_R, in_ea, MAHA_THRESH_2, 0);
}

void kinematic_update_3(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea) {
  (void)update_core(1, in_x, in_P, h_3, H_3, in_z, in_R, in_ea, MAHA_THRESH_3, 1);
}

void kinematic_update_4(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea) {
  (void)update_core(2, in_x, in_P, h_4, H_4, in_z, in_R, in_ea, MAHA_THRESH_4, 0);
}
