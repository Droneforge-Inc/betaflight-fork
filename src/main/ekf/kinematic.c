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
static const float MAHA_THRESH_3 = 5.9914645471079808f;
static const float MAHA_THRESH_4 = 5.9914645471079808f;


/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
static void err_fun(float *nom_x, float *delta_x, float *out_3659427682202181060) {
   out_3659427682202181060[0] = delta_x[0] + nom_x[0];
   out_3659427682202181060[1] = delta_x[1] + nom_x[1];
   out_3659427682202181060[2] = delta_x[2] + nom_x[2];
   out_3659427682202181060[3] = delta_x[3] + nom_x[3];
   out_3659427682202181060[4] = delta_x[4] + nom_x[4];
   out_3659427682202181060[5] = delta_x[5] + nom_x[5];
   out_3659427682202181060[6] = delta_x[6] + nom_x[6];
   out_3659427682202181060[7] = delta_x[7] + nom_x[7];
   out_3659427682202181060[8] = delta_x[8] + nom_x[8];
   out_3659427682202181060[9] = delta_x[9] + nom_x[9];
}
static void inv_err_fun(float *nom_x, float *true_x, float *out_85100853329552610) {
   out_85100853329552610[0] = -nom_x[0] + true_x[0];
   out_85100853329552610[1] = -nom_x[1] + true_x[1];
   out_85100853329552610[2] = -nom_x[2] + true_x[2];
   out_85100853329552610[3] = -nom_x[3] + true_x[3];
   out_85100853329552610[4] = -nom_x[4] + true_x[4];
   out_85100853329552610[5] = -nom_x[5] + true_x[5];
   out_85100853329552610[6] = -nom_x[6] + true_x[6];
   out_85100853329552610[7] = -nom_x[7] + true_x[7];
   out_85100853329552610[8] = -nom_x[8] + true_x[8];
   out_85100853329552610[9] = -nom_x[9] + true_x[9];
}
static void H_mod_fun(float *state, float *out_5295332717199273029) {
  (void)state;
   out_5295332717199273029[0] = 1.0;
   out_5295332717199273029[1] = 0.0;
   out_5295332717199273029[2] = 0.0;
   out_5295332717199273029[3] = 0.0;
   out_5295332717199273029[4] = 0.0;
   out_5295332717199273029[5] = 0.0;
   out_5295332717199273029[6] = 0.0;
   out_5295332717199273029[7] = 0.0;
   out_5295332717199273029[8] = 0.0;
   out_5295332717199273029[9] = 0.0;
   out_5295332717199273029[10] = 0.0;
   out_5295332717199273029[11] = 1.0;
   out_5295332717199273029[12] = 0.0;
   out_5295332717199273029[13] = 0.0;
   out_5295332717199273029[14] = 0.0;
   out_5295332717199273029[15] = 0.0;
   out_5295332717199273029[16] = 0.0;
   out_5295332717199273029[17] = 0.0;
   out_5295332717199273029[18] = 0.0;
   out_5295332717199273029[19] = 0.0;
   out_5295332717199273029[20] = 0.0;
   out_5295332717199273029[21] = 0.0;
   out_5295332717199273029[22] = 1.0;
   out_5295332717199273029[23] = 0.0;
   out_5295332717199273029[24] = 0.0;
   out_5295332717199273029[25] = 0.0;
   out_5295332717199273029[26] = 0.0;
   out_5295332717199273029[27] = 0.0;
   out_5295332717199273029[28] = 0.0;
   out_5295332717199273029[29] = 0.0;
   out_5295332717199273029[30] = 0.0;
   out_5295332717199273029[31] = 0.0;
   out_5295332717199273029[32] = 0.0;
   out_5295332717199273029[33] = 1.0;
   out_5295332717199273029[34] = 0.0;
   out_5295332717199273029[35] = 0.0;
   out_5295332717199273029[36] = 0.0;
   out_5295332717199273029[37] = 0.0;
   out_5295332717199273029[38] = 0.0;
   out_5295332717199273029[39] = 0.0;
   out_5295332717199273029[40] = 0.0;
   out_5295332717199273029[41] = 0.0;
   out_5295332717199273029[42] = 0.0;
   out_5295332717199273029[43] = 0.0;
   out_5295332717199273029[44] = 1.0;
   out_5295332717199273029[45] = 0.0;
   out_5295332717199273029[46] = 0.0;
   out_5295332717199273029[47] = 0.0;
   out_5295332717199273029[48] = 0.0;
   out_5295332717199273029[49] = 0.0;
   out_5295332717199273029[50] = 0.0;
   out_5295332717199273029[51] = 0.0;
   out_5295332717199273029[52] = 0.0;
   out_5295332717199273029[53] = 0.0;
   out_5295332717199273029[54] = 0.0;
   out_5295332717199273029[55] = 1.0;
   out_5295332717199273029[56] = 0.0;
   out_5295332717199273029[57] = 0.0;
   out_5295332717199273029[58] = 0.0;
   out_5295332717199273029[59] = 0.0;
   out_5295332717199273029[60] = 0.0;
   out_5295332717199273029[61] = 0.0;
   out_5295332717199273029[62] = 0.0;
   out_5295332717199273029[63] = 0.0;
   out_5295332717199273029[64] = 0.0;
   out_5295332717199273029[65] = 0.0;
   out_5295332717199273029[66] = 1.0;
   out_5295332717199273029[67] = 0.0;
   out_5295332717199273029[68] = 0.0;
   out_5295332717199273029[69] = 0.0;
   out_5295332717199273029[70] = 0.0;
   out_5295332717199273029[71] = 0.0;
   out_5295332717199273029[72] = 0.0;
   out_5295332717199273029[73] = 0.0;
   out_5295332717199273029[74] = 0.0;
   out_5295332717199273029[75] = 0.0;
   out_5295332717199273029[76] = 0.0;
   out_5295332717199273029[77] = 1.0;
   out_5295332717199273029[78] = 0.0;
   out_5295332717199273029[79] = 0.0;
   out_5295332717199273029[80] = 0.0;
   out_5295332717199273029[81] = 0.0;
   out_5295332717199273029[82] = 0.0;
   out_5295332717199273029[83] = 0.0;
   out_5295332717199273029[84] = 0.0;
   out_5295332717199273029[85] = 0.0;
   out_5295332717199273029[86] = 0.0;
   out_5295332717199273029[87] = 0.0;
   out_5295332717199273029[88] = 1.0;
   out_5295332717199273029[89] = 0.0;
   out_5295332717199273029[90] = 0.0;
   out_5295332717199273029[91] = 0.0;
   out_5295332717199273029[92] = 0.0;
   out_5295332717199273029[93] = 0.0;
   out_5295332717199273029[94] = 0.0;
   out_5295332717199273029[95] = 0.0;
   out_5295332717199273029[96] = 0.0;
   out_5295332717199273029[97] = 0.0;
   out_5295332717199273029[98] = 0.0;
   out_5295332717199273029[99] = 1.0;
}
static void f_fun(float *state, float *u, float dt, float *out_3314950734666552737) {
   out_3314950734666552737[0] = (1.0/2.0)*powf(dt, 2)*((2*u[3]*u[5] + 2*u[4]*u[6])*(-state[9] + u[2]) + (-2*u[3]*u[6] + 2*u[4]*u[5])*(-state[8] + u[1]) + (-state[7] + u[0])*(powf(u[3], 2) + powf(u[4], 2) - powf(u[5], 2) - powf(u[6], 2))) + dt*state[3] + state[0];
   out_3314950734666552737[1] = (1.0/2.0)*powf(dt, 2)*((-2*u[3]*u[4] + 2*u[5]*u[6])*(-state[9] + u[2]) + (2*u[3]*u[6] + 2*u[4]*u[5])*(-state[7] + u[0]) + (-state[8] + u[1])*(powf(u[3], 2) - powf(u[4], 2) + powf(u[5], 2) - powf(u[6], 2))) + dt*state[4] + state[1];
   out_3314950734666552737[2] = (1.0/2.0)*powf(dt, 2)*((2*u[3]*u[4] + 2*u[5]*u[6])*(-state[8] + u[1]) + (-2*u[3]*u[5] + 2*u[4]*u[6])*(-state[7] + u[0]) + (-state[9] + u[2])*(powf(u[3], 2) - powf(u[4], 2) - powf(u[5], 2) + powf(u[6], 2)) - 9.8066499999999994) + dt*state[5] + state[2];
   out_3314950734666552737[3] = dt*((2*u[3]*u[5] + 2*u[4]*u[6])*(-state[9] + u[2]) + (-2*u[3]*u[6] + 2*u[4]*u[5])*(-state[8] + u[1]) + (-state[7] + u[0])*(powf(u[3], 2) + powf(u[4], 2) - powf(u[5], 2) - powf(u[6], 2))) + state[3];
   out_3314950734666552737[4] = dt*((-2*u[3]*u[4] + 2*u[5]*u[6])*(-state[9] + u[2]) + (2*u[3]*u[6] + 2*u[4]*u[5])*(-state[7] + u[0]) + (-state[8] + u[1])*(powf(u[3], 2) - powf(u[4], 2) + powf(u[5], 2) - powf(u[6], 2))) + state[4];
   out_3314950734666552737[5] = dt*((2*u[3]*u[4] + 2*u[5]*u[6])*(-state[8] + u[1]) + (-2*u[3]*u[5] + 2*u[4]*u[6])*(-state[7] + u[0]) + (-state[9] + u[2])*(powf(u[3], 2) - powf(u[4], 2) - powf(u[5], 2) + powf(u[6], 2)) - 9.8066499999999994) + state[5];
   out_3314950734666552737[6] = state[6];
   out_3314950734666552737[7] = state[7];
   out_3314950734666552737[8] = state[8];
   out_3314950734666552737[9] = state[9];
}
static void F_fun(float *state, float *u, float dt, float *out_3367807419579831087) {
  (void)state;
   out_3367807419579831087[0] = 1;
   out_3367807419579831087[1] = 0;
   out_3367807419579831087[2] = 0;
   out_3367807419579831087[3] = dt;
   out_3367807419579831087[4] = 0;
   out_3367807419579831087[5] = 0;
   out_3367807419579831087[6] = 0;
   out_3367807419579831087[7] = (1.0/2.0)*powf(dt, 2)*(-powf(u[3], 2) - powf(u[4], 2) + powf(u[5], 2) + powf(u[6], 2));
   out_3367807419579831087[8] = (1.0/2.0)*powf(dt, 2)*(2*u[3]*u[6] - 2*u[4]*u[5]);
   out_3367807419579831087[9] = (1.0/2.0)*powf(dt, 2)*(-2*u[3]*u[5] - 2*u[4]*u[6]);
   out_3367807419579831087[10] = 0;
   out_3367807419579831087[11] = 1;
   out_3367807419579831087[12] = 0;
   out_3367807419579831087[13] = 0;
   out_3367807419579831087[14] = dt;
   out_3367807419579831087[15] = 0;
   out_3367807419579831087[16] = 0;
   out_3367807419579831087[17] = (1.0/2.0)*powf(dt, 2)*(-2*u[3]*u[6] - 2*u[4]*u[5]);
   out_3367807419579831087[18] = (1.0/2.0)*powf(dt, 2)*(-powf(u[3], 2) + powf(u[4], 2) - powf(u[5], 2) + powf(u[6], 2));
   out_3367807419579831087[19] = (1.0/2.0)*powf(dt, 2)*(2*u[3]*u[4] - 2*u[5]*u[6]);
   out_3367807419579831087[20] = 0;
   out_3367807419579831087[21] = 0;
   out_3367807419579831087[22] = 1;
   out_3367807419579831087[23] = 0;
   out_3367807419579831087[24] = 0;
   out_3367807419579831087[25] = dt;
   out_3367807419579831087[26] = 0;
   out_3367807419579831087[27] = (1.0/2.0)*powf(dt, 2)*(2*u[3]*u[5] - 2*u[4]*u[6]);
   out_3367807419579831087[28] = (1.0/2.0)*powf(dt, 2)*(-2*u[3]*u[4] - 2*u[5]*u[6]);
   out_3367807419579831087[29] = (1.0/2.0)*powf(dt, 2)*(-powf(u[3], 2) + powf(u[4], 2) + powf(u[5], 2) - powf(u[6], 2));
   out_3367807419579831087[30] = 0;
   out_3367807419579831087[31] = 0;
   out_3367807419579831087[32] = 0;
   out_3367807419579831087[33] = 1;
   out_3367807419579831087[34] = 0;
   out_3367807419579831087[35] = 0;
   out_3367807419579831087[36] = 0;
   out_3367807419579831087[37] = dt*(-powf(u[3], 2) - powf(u[4], 2) + powf(u[5], 2) + powf(u[6], 2));
   out_3367807419579831087[38] = dt*(2*u[3]*u[6] - 2*u[4]*u[5]);
   out_3367807419579831087[39] = dt*(-2*u[3]*u[5] - 2*u[4]*u[6]);
   out_3367807419579831087[40] = 0;
   out_3367807419579831087[41] = 0;
   out_3367807419579831087[42] = 0;
   out_3367807419579831087[43] = 0;
   out_3367807419579831087[44] = 1;
   out_3367807419579831087[45] = 0;
   out_3367807419579831087[46] = 0;
   out_3367807419579831087[47] = dt*(-2*u[3]*u[6] - 2*u[4]*u[5]);
   out_3367807419579831087[48] = dt*(-powf(u[3], 2) + powf(u[4], 2) - powf(u[5], 2) + powf(u[6], 2));
   out_3367807419579831087[49] = dt*(2*u[3]*u[4] - 2*u[5]*u[6]);
   out_3367807419579831087[50] = 0;
   out_3367807419579831087[51] = 0;
   out_3367807419579831087[52] = 0;
   out_3367807419579831087[53] = 0;
   out_3367807419579831087[54] = 0;
   out_3367807419579831087[55] = 1;
   out_3367807419579831087[56] = 0;
   out_3367807419579831087[57] = dt*(2*u[3]*u[5] - 2*u[4]*u[6]);
   out_3367807419579831087[58] = dt*(-2*u[3]*u[4] - 2*u[5]*u[6]);
   out_3367807419579831087[59] = dt*(-powf(u[3], 2) + powf(u[4], 2) + powf(u[5], 2) - powf(u[6], 2));
   out_3367807419579831087[60] = 0;
   out_3367807419579831087[61] = 0;
   out_3367807419579831087[62] = 0;
   out_3367807419579831087[63] = 0;
   out_3367807419579831087[64] = 0;
   out_3367807419579831087[65] = 0;
   out_3367807419579831087[66] = 1;
   out_3367807419579831087[67] = 0;
   out_3367807419579831087[68] = 0;
   out_3367807419579831087[69] = 0;
   out_3367807419579831087[70] = 0;
   out_3367807419579831087[71] = 0;
   out_3367807419579831087[72] = 0;
   out_3367807419579831087[73] = 0;
   out_3367807419579831087[74] = 0;
   out_3367807419579831087[75] = 0;
   out_3367807419579831087[76] = 0;
   out_3367807419579831087[77] = 1;
   out_3367807419579831087[78] = 0;
   out_3367807419579831087[79] = 0;
   out_3367807419579831087[80] = 0;
   out_3367807419579831087[81] = 0;
   out_3367807419579831087[82] = 0;
   out_3367807419579831087[83] = 0;
   out_3367807419579831087[84] = 0;
   out_3367807419579831087[85] = 0;
   out_3367807419579831087[86] = 0;
   out_3367807419579831087[87] = 0;
   out_3367807419579831087[88] = 1;
   out_3367807419579831087[89] = 0;
   out_3367807419579831087[90] = 0;
   out_3367807419579831087[91] = 0;
   out_3367807419579831087[92] = 0;
   out_3367807419579831087[93] = 0;
   out_3367807419579831087[94] = 0;
   out_3367807419579831087[95] = 0;
   out_3367807419579831087[96] = 0;
   out_3367807419579831087[97] = 0;
   out_3367807419579831087[98] = 0;
   out_3367807419579831087[99] = 1;
}
static void Q_fun(float *state, float *u, float dt, float *out_107151997113418972) {
  (void)state;
  (void)u;
   out_107151997113418972[0] = 0.25*powf(dt, 4);
   out_107151997113418972[1] = 0;
   out_107151997113418972[2] = 0;
   out_107151997113418972[3] = 0.5*powf(dt, 3);
   out_107151997113418972[4] = 0;
   out_107151997113418972[5] = 0;
   out_107151997113418972[6] = 0;
   out_107151997113418972[7] = 0;
   out_107151997113418972[8] = 0;
   out_107151997113418972[9] = 0;
   out_107151997113418972[10] = 0;
   out_107151997113418972[11] = 0.25*powf(dt, 4);
   out_107151997113418972[12] = 0;
   out_107151997113418972[13] = 0;
   out_107151997113418972[14] = 0.5*powf(dt, 3);
   out_107151997113418972[15] = 0;
   out_107151997113418972[16] = 0;
   out_107151997113418972[17] = 0;
   out_107151997113418972[18] = 0;
   out_107151997113418972[19] = 0;
   out_107151997113418972[20] = 0;
   out_107151997113418972[21] = 0;
   out_107151997113418972[22] = 0.25*powf(dt, 4);
   out_107151997113418972[23] = 0;
   out_107151997113418972[24] = 0;
   out_107151997113418972[25] = 0.5*powf(dt, 3);
   out_107151997113418972[26] = 0;
   out_107151997113418972[27] = 0;
   out_107151997113418972[28] = 0;
   out_107151997113418972[29] = 0;
   out_107151997113418972[30] = 0.5*powf(dt, 3);
   out_107151997113418972[31] = 0;
   out_107151997113418972[32] = 0;
   out_107151997113418972[33] = 1.0*powf(dt, 2);
   out_107151997113418972[34] = 0;
   out_107151997113418972[35] = 0;
   out_107151997113418972[36] = 0;
   out_107151997113418972[37] = 0;
   out_107151997113418972[38] = 0;
   out_107151997113418972[39] = 0;
   out_107151997113418972[40] = 0;
   out_107151997113418972[41] = 0.5*powf(dt, 3);
   out_107151997113418972[42] = 0;
   out_107151997113418972[43] = 0;
   out_107151997113418972[44] = 1.0*powf(dt, 2);
   out_107151997113418972[45] = 0;
   out_107151997113418972[46] = 0;
   out_107151997113418972[47] = 0;
   out_107151997113418972[48] = 0;
   out_107151997113418972[49] = 0;
   out_107151997113418972[50] = 0;
   out_107151997113418972[51] = 0;
   out_107151997113418972[52] = 0.5*powf(dt, 3);
   out_107151997113418972[53] = 0;
   out_107151997113418972[54] = 0;
   out_107151997113418972[55] = 1.0*powf(dt, 2);
   out_107151997113418972[56] = 0;
   out_107151997113418972[57] = 0;
   out_107151997113418972[58] = 0;
   out_107151997113418972[59] = 0;
   out_107151997113418972[60] = 0;
   out_107151997113418972[61] = 0;
   out_107151997113418972[62] = 0;
   out_107151997113418972[63] = 0;
   out_107151997113418972[64] = 0;
   out_107151997113418972[65] = 0;
   out_107151997113418972[66] = 0.0001*dt;
   out_107151997113418972[67] = 0;
   out_107151997113418972[68] = 0;
   out_107151997113418972[69] = 0;
   out_107151997113418972[70] = 0;
   out_107151997113418972[71] = 0;
   out_107151997113418972[72] = 0;
   out_107151997113418972[73] = 0;
   out_107151997113418972[74] = 0;
   out_107151997113418972[75] = 0;
   out_107151997113418972[76] = 0;
   out_107151997113418972[77] = 2.5000000000000001e-5*dt;
   out_107151997113418972[78] = 0;
   out_107151997113418972[79] = 0;
   out_107151997113418972[80] = 0;
   out_107151997113418972[81] = 0;
   out_107151997113418972[82] = 0;
   out_107151997113418972[83] = 0;
   out_107151997113418972[84] = 0;
   out_107151997113418972[85] = 0;
   out_107151997113418972[86] = 0;
   out_107151997113418972[87] = 0;
   out_107151997113418972[88] = 2.5000000000000001e-5*dt;
   out_107151997113418972[89] = 0;
   out_107151997113418972[90] = 0;
   out_107151997113418972[91] = 0;
   out_107151997113418972[92] = 0;
   out_107151997113418972[93] = 0;
   out_107151997113418972[94] = 0;
   out_107151997113418972[95] = 0;
   out_107151997113418972[96] = 0;
   out_107151997113418972[97] = 0;
   out_107151997113418972[98] = 0;
   out_107151997113418972[99] = 2.5000000000000001e-5*dt;
}
static void h_2(float *state, float *unused, float *out_790282607527524595) {
  (void)unused;
   out_790282607527524595[0] = state[2];
}
static void H_2(float *state, float *unused, float *out_3663466633305576164) {
  (void)state;
  (void)unused;
   out_3663466633305576164[0] = 0;
   out_3663466633305576164[1] = 0;
   out_3663466633305576164[2] = 1;
   out_3663466633305576164[3] = 0;
   out_3663466633305576164[4] = 0;
   out_3663466633305576164[5] = 0;
   out_3663466633305576164[6] = 0;
   out_3663466633305576164[7] = 0;
   out_3663466633305576164[8] = 0;
   out_3663466633305576164[9] = 0;
}
static void h_3(float *state, float *unused, float *out_931129733195384793) {
  (void)unused;
   out_931129733195384793[0] = state[2] + state[6];
   out_931129733195384793[1] = state[5];
}
static void H_3(float *state, float *unused, float *out_5102280481451938197) {
  (void)state;
  (void)unused;
   out_5102280481451938197[0] = 0;
   out_5102280481451938197[1] = 0;
   out_5102280481451938197[2] = 1;
   out_5102280481451938197[3] = 0;
   out_5102280481451938197[4] = 0;
   out_5102280481451938197[5] = 0;
   out_5102280481451938197[6] = 1;
   out_5102280481451938197[7] = 0;
   out_5102280481451938197[8] = 0;
   out_5102280481451938197[9] = 0;
   out_5102280481451938197[10] = 0;
   out_5102280481451938197[11] = 0;
   out_5102280481451938197[12] = 0;
   out_5102280481451938197[13] = 0;
   out_5102280481451938197[14] = 0;
   out_5102280481451938197[15] = 1;
   out_5102280481451938197[16] = 0;
   out_5102280481451938197[17] = 0;
   out_5102280481451938197[18] = 0;
   out_5102280481451938197[19] = 0;
}
static void h_4(float *state, float *flow_q, float *out_4031177123136809253) {
   out_4031177123136809253[0] = (-2*flow_q[0]*flow_q[2] + 2*flow_q[1]*flow_q[3])*state[5] + (2*flow_q[0]*flow_q[3] + 2*flow_q[1]*flow_q[2])*state[4] + (powf(flow_q[0], 2) + powf(flow_q[1], 2) - powf(flow_q[2], 2) - powf(flow_q[3], 2))*state[3];
   out_4031177123136809253[1] = (2*flow_q[0]*flow_q[1] + 2*flow_q[2]*flow_q[3])*state[5] + (-2*flow_q[0]*flow_q[3] + 2*flow_q[1]*flow_q[2])*state[3] + (powf(flow_q[0], 2) - powf(flow_q[1], 2) + powf(flow_q[2], 2) - powf(flow_q[3], 2))*state[4];
}
static void H_4(float *state, float *flow_q, float *out_2822554916459715235) {
  (void)state;
   out_2822554916459715235[0] = 0;
   out_2822554916459715235[1] = 0;
   out_2822554916459715235[2] = 0;
   out_2822554916459715235[3] = powf(flow_q[0], 2) + powf(flow_q[1], 2) - powf(flow_q[2], 2) - powf(flow_q[3], 2);
   out_2822554916459715235[4] = 2*flow_q[0]*flow_q[3] + 2*flow_q[1]*flow_q[2];
   out_2822554916459715235[5] = -2*flow_q[0]*flow_q[2] + 2*flow_q[1]*flow_q[3];
   out_2822554916459715235[6] = 0;
   out_2822554916459715235[7] = 0;
   out_2822554916459715235[8] = 0;
   out_2822554916459715235[9] = 0;
   out_2822554916459715235[10] = 0;
   out_2822554916459715235[11] = 0;
   out_2822554916459715235[12] = 0;
   out_2822554916459715235[13] = -2*flow_q[0]*flow_q[3] + 2*flow_q[1]*flow_q[2];
   out_2822554916459715235[14] = powf(flow_q[0], 2) - powf(flow_q[1], 2) + powf(flow_q[2], 2) - powf(flow_q[3], 2);
   out_2822554916459715235[15] = 2*flow_q[0]*flow_q[1] + 2*flow_q[2]*flow_q[3];
   out_2822554916459715235[16] = 0;
   out_2822554916459715235[17] = 0;
   out_2822554916459715235[18] = 0;
   out_2822554916459715235[19] = 0;
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
  (void)update_core(2, in_x, in_P, h_3, H_3, in_z, in_R, in_ea, MAHA_THRESH_3, 1);
}

void kinematic_update_4(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea) {
  (void)update_core(2, in_x, in_P, h_4, H_4, in_z, in_R, in_ea, MAHA_THRESH_4, 0);
}
