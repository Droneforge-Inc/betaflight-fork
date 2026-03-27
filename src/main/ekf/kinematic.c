#include "kinematic.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define DIM 10
#define EDIM 10
#define MEDIM 10
#define MAX_ZDIM 2
#define H_MOD_IS_IDENTITY 1
#define Q_SYM_IS_DISCRETE 1
typedef void (*ObsFun)(float *, float *, float *);

static const float MAHA_THRESH_2 = 3.8414588206941227f;
static const float MAHA_THRESH_3 = 3.8414588206941227f;
static const float MAHA_THRESH_4 = 5.9914645471079808f;
static const float MAHA_THRESH_5 = 3.8414588206941227f;
static const float MAHA_THRESH_6 = 5.9914645471079808f;
static const float MAHA_THRESH_7 = 5.9914645471079808f;


static void err_fun(float *nom_x, float *delta_x, float *out) {
  out[0] = delta_x[0] + nom_x[0];
  out[1] = delta_x[1] + nom_x[1];
  out[2] = delta_x[2] + nom_x[2];
  out[3] = delta_x[3] + nom_x[3];
  out[4] = delta_x[4] + nom_x[4];
  out[5] = delta_x[5] + nom_x[5];
  out[6] = delta_x[6] + nom_x[6];
  out[7] = delta_x[7] + nom_x[7];
  out[8] = delta_x[8] + nom_x[8];
  out[9] = delta_x[9] + nom_x[9];
}

static void inv_err_fun(float *nom_x, float *true_x, float *out) {
  out[0] = -nom_x[0] + true_x[0];
  out[1] = -nom_x[1] + true_x[1];
  out[2] = -nom_x[2] + true_x[2];
  out[3] = -nom_x[3] + true_x[3];
  out[4] = -nom_x[4] + true_x[4];
  out[5] = -nom_x[5] + true_x[5];
  out[6] = -nom_x[6] + true_x[6];
  out[7] = -nom_x[7] + true_x[7];
  out[8] = -nom_x[8] + true_x[8];
  out[9] = -nom_x[9] + true_x[9];
}

static void H_mod_fun(float *state, float *out) {
  (void)state;
  memset(out, 0, 100 * sizeof(float));
  out[0] = 1.0F;
  out[11] = 1.0F;
  out[22] = 1.0F;
  out[33] = 1.0F;
  out[44] = 1.0F;
  out[55] = 1.0F;
  out[66] = 1.0F;
  out[77] = 1.0F;
  out[88] = 1.0F;
  out[99] = 1.0F;
}

static void f_fun(float *state, float *u, float dt, float *out) {
  const float cse_tmp_0 = -state[9] + u[2];
  const float cse_tmp_1 = 2*u[3];
  const float cse_tmp_2 = cse_tmp_1*u[5];
  const float cse_tmp_3 = 2*u[4];
  const float cse_tmp_4 = -state[8] + u[1];
  const float cse_tmp_5 = cse_tmp_1*u[6];
  const float cse_tmp_6 = -state[7] + u[0];
  const float cse_tmp_7 = (u[4]) * (u[4]);
  const float cse_tmp_8 = (u[5]) * (u[5]);
  const float cse_tmp_9 = -cse_tmp_8;
  const float cse_tmp_10 = (u[3]) * (u[3]);
  const float cse_tmp_11 = (u[6]) * (u[6]);
  const float cse_tmp_12 = cse_tmp_10 - cse_tmp_11;
  const float cse_tmp_13 = cse_tmp_0*(cse_tmp_2 + cse_tmp_3*u[6]) + cse_tmp_4*(-cse_tmp_5 + 2*u[4]*u[5]) + cse_tmp_6*(cse_tmp_12 + cse_tmp_7 + cse_tmp_9);
  const float cse_tmp_14 = (1.0F/2.0F)*(dt) * (dt);
  const float cse_tmp_15 = cse_tmp_1*u[4];
  const float cse_tmp_16 = -cse_tmp_7;
  const float cse_tmp_17 = cse_tmp_0*(-cse_tmp_15 + 2*u[5]*u[6]) + cse_tmp_4*(cse_tmp_12 + cse_tmp_16 + cse_tmp_8) + cse_tmp_6*(cse_tmp_3*u[5] + cse_tmp_5);
  const float cse_tmp_18 = cse_tmp_0*(cse_tmp_10 + cse_tmp_11 + cse_tmp_16 + cse_tmp_9) + cse_tmp_4*(cse_tmp_15 + 2*u[5]*u[6]) + cse_tmp_6*(-cse_tmp_2 + 2*u[4]*u[6]) - 9.80665F;
  out[0] = cse_tmp_13*cse_tmp_14 + dt*state[3] + state[0];
  out[1] = cse_tmp_14*cse_tmp_17 + dt*state[4] + state[1];
  out[2] = cse_tmp_14*cse_tmp_18 + dt*state[5] + state[2];
  out[3] = cse_tmp_13*dt + state[3];
  out[4] = cse_tmp_17*dt + state[4];
  out[5] = cse_tmp_18*dt + state[5];
  out[6] = state[6];
  out[7] = state[7];
  out[8] = state[8];
  out[9] = state[9];
}

static void F_fun(float *state, float *u, float dt, float *out) {
  (void)state;
  const float cse_tmp_0 = (u[4]) * (u[4]);
  const float cse_tmp_1 = (u[5]) * (u[5]);
  const float cse_tmp_2 = -cse_tmp_1;
  const float cse_tmp_3 = (u[3]) * (u[3]);
  const float cse_tmp_4 = (u[6]) * (u[6]);
  const float cse_tmp_5 = cse_tmp_3 - cse_tmp_4;
  const float cse_tmp_6 = -cse_tmp_0 - cse_tmp_2 - cse_tmp_5;
  const float cse_tmp_7 = (1.0F/2.0F)*(dt) * (dt);
  const float cse_tmp_8 = 2*u[6];
  const float cse_tmp_9 = cse_tmp_8*u[3];
  const float cse_tmp_10 = 2*u[5];
  const float cse_tmp_11 = cse_tmp_10*u[4];
  const float cse_tmp_12 = -cse_tmp_11 + cse_tmp_9;
  const float cse_tmp_13 = cse_tmp_10*u[3];
  const float cse_tmp_14 = cse_tmp_8*u[4];
  const float cse_tmp_15 = -cse_tmp_13 - cse_tmp_14;
  const float cse_tmp_16 = -cse_tmp_11 - cse_tmp_9;
  const float cse_tmp_17 = -cse_tmp_0;
  const float cse_tmp_18 = -cse_tmp_1 - cse_tmp_17 - cse_tmp_5;
  const float cse_tmp_19 = 2*u[3]*u[4];
  const float cse_tmp_20 = cse_tmp_10*u[6];
  const float cse_tmp_21 = cse_tmp_19 - cse_tmp_20;
  const float cse_tmp_22 = cse_tmp_13 - cse_tmp_14;
  const float cse_tmp_23 = -cse_tmp_19 - cse_tmp_20;
  const float cse_tmp_24 = -cse_tmp_17 - cse_tmp_2 - cse_tmp_3 - cse_tmp_4;
  out[0] = 1;
  out[1] = 0;
  out[2] = 0;
  out[3] = dt;
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = cse_tmp_6*cse_tmp_7;
  out[8] = cse_tmp_12*cse_tmp_7;
  out[9] = cse_tmp_15*cse_tmp_7;
  out[10] = 0;
  out[11] = 1;
  out[12] = 0;
  out[13] = 0;
  out[14] = dt;
  out[15] = 0;
  out[16] = 0;
  out[17] = cse_tmp_16*cse_tmp_7;
  out[18] = cse_tmp_18*cse_tmp_7;
  out[19] = cse_tmp_21*cse_tmp_7;
  out[20] = 0;
  out[21] = 0;
  out[22] = 1;
  out[23] = 0;
  out[24] = 0;
  out[25] = dt;
  out[26] = 0;
  out[27] = cse_tmp_22*cse_tmp_7;
  out[28] = cse_tmp_23*cse_tmp_7;
  out[29] = cse_tmp_24*cse_tmp_7;
  out[30] = 0;
  out[31] = 0;
  out[32] = 0;
  out[33] = 1;
  out[34] = 0;
  out[35] = 0;
  out[36] = 0;
  out[37] = cse_tmp_6*dt;
  out[38] = cse_tmp_12*dt;
  out[39] = cse_tmp_15*dt;
  out[40] = 0;
  out[41] = 0;
  out[42] = 0;
  out[43] = 0;
  out[44] = 1;
  out[45] = 0;
  out[46] = 0;
  out[47] = cse_tmp_16*dt;
  out[48] = cse_tmp_18*dt;
  out[49] = cse_tmp_21*dt;
  out[50] = 0;
  out[51] = 0;
  out[52] = 0;
  out[53] = 0;
  out[54] = 0;
  out[55] = 1;
  out[56] = 0;
  out[57] = cse_tmp_22*dt;
  out[58] = cse_tmp_23*dt;
  out[59] = cse_tmp_24*dt;
  out[60] = 0;
  out[61] = 0;
  out[62] = 0;
  out[63] = 0;
  out[64] = 0;
  out[65] = 0;
  out[66] = 1;
  out[67] = 0;
  out[68] = 0;
  out[69] = 0;
  out[70] = 0;
  out[71] = 0;
  out[72] = 0;
  out[73] = 0;
  out[74] = 0;
  out[75] = 0;
  out[76] = 0;
  out[77] = 1;
  out[78] = 0;
  out[79] = 0;
  out[80] = 0;
  out[81] = 0;
  out[82] = 0;
  out[83] = 0;
  out[84] = 0;
  out[85] = 0;
  out[86] = 0;
  out[87] = 0;
  out[88] = 1;
  out[89] = 0;
  out[90] = 0;
  out[91] = 0;
  out[92] = 0;
  out[93] = 0;
  out[94] = 0;
  out[95] = 0;
  out[96] = 0;
  out[97] = 0;
  out[98] = 0;
  out[99] = 1;
}

static void Q_fun(float *state, float *u, float dt, float *out) {
  (void)state;
  (void)u;
  const float cse_tmp_0 = 0.25F*(dt) * (dt) * (dt) * (dt);
  const float cse_tmp_1 = 0.5F*(dt) * (dt) * (dt);
  const float cse_tmp_2 = 1.0F*(dt) * (dt);
  const float cse_tmp_3 = 2.5e-5F*dt;
  out[0] = cse_tmp_0;
  out[1] = 0;
  out[2] = 0;
  out[3] = cse_tmp_1;
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 0;
  out[11] = cse_tmp_0;
  out[12] = 0;
  out[13] = 0;
  out[14] = cse_tmp_1;
  out[15] = 0;
  out[16] = 0;
  out[17] = 0;
  out[18] = 0;
  out[19] = 0;
  out[20] = 0;
  out[21] = 0;
  out[22] = cse_tmp_0;
  out[23] = 0;
  out[24] = 0;
  out[25] = cse_tmp_1;
  out[26] = 0;
  out[27] = 0;
  out[28] = 0;
  out[29] = 0;
  out[30] = cse_tmp_1;
  out[31] = 0;
  out[32] = 0;
  out[33] = cse_tmp_2;
  out[34] = 0;
  out[35] = 0;
  out[36] = 0;
  out[37] = 0;
  out[38] = 0;
  out[39] = 0;
  out[40] = 0;
  out[41] = cse_tmp_1;
  out[42] = 0;
  out[43] = 0;
  out[44] = cse_tmp_2;
  out[45] = 0;
  out[46] = 0;
  out[47] = 0;
  out[48] = 0;
  out[49] = 0;
  out[50] = 0;
  out[51] = 0;
  out[52] = cse_tmp_1;
  out[53] = 0;
  out[54] = 0;
  out[55] = cse_tmp_2;
  out[56] = 0;
  out[57] = 0;
  out[58] = 0;
  out[59] = 0;
  out[60] = 0;
  out[61] = 0;
  out[62] = 0;
  out[63] = 0;
  out[64] = 0;
  out[65] = 0;
  out[66] = 0.0001F*dt;
  out[67] = 0;
  out[68] = 0;
  out[69] = 0;
  out[70] = 0;
  out[71] = 0;
  out[72] = 0;
  out[73] = 0;
  out[74] = 0;
  out[75] = 0;
  out[76] = 0;
  out[77] = cse_tmp_3;
  out[78] = 0;
  out[79] = 0;
  out[80] = 0;
  out[81] = 0;
  out[82] = 0;
  out[83] = 0;
  out[84] = 0;
  out[85] = 0;
  out[86] = 0;
  out[87] = 0;
  out[88] = cse_tmp_3;
  out[89] = 0;
  out[90] = 0;
  out[91] = 0;
  out[92] = 0;
  out[93] = 0;
  out[94] = 0;
  out[95] = 0;
  out[96] = 0;
  out[97] = 0;
  out[98] = 0;
  out[99] = cse_tmp_3;
}

static void h_2(float *state, float *unused, float *out) {
  (void)unused;
  out[0] = state[2];
}

static void H_2(float *state, float *unused, float *out) {
  (void)state;
  (void)unused;
  out[0] = 0;
  out[1] = 0;
  out[2] = 1;
  out[3] = 0;
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
}

static void h_3(float *state, float *unused, float *out) {
  (void)unused;
  out[0] = state[2] + state[6];
}

static void H_3(float *state, float *unused, float *out) {
  (void)state;
  (void)unused;
  out[0] = 0;
  out[1] = 0;
  out[2] = 1;
  out[3] = 0;
  out[4] = 0;
  out[5] = 0;
  out[6] = 1;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
}

static void h_4(float *state, float *flow_q, float *out) {
  const float cse_tmp_0 = 2*flow_q[0];
  const float cse_tmp_1 = cse_tmp_0*flow_q[3];
  const float cse_tmp_2 = (flow_q[1]) * (flow_q[1]);
  const float cse_tmp_3 = (flow_q[2]) * (flow_q[2]);
  const float cse_tmp_4 = (flow_q[0]) * (flow_q[0]) - (flow_q[3]) * (flow_q[3]);
  out[0] = (cse_tmp_1 + 2*flow_q[1]*flow_q[2])*state[4] + (-cse_tmp_0*flow_q[2] + 2*flow_q[1]*flow_q[3])*state[5] + (cse_tmp_2 - cse_tmp_3 + cse_tmp_4)*state[3];
  out[1] = (-cse_tmp_1 + 2*flow_q[1]*flow_q[2])*state[3] + (cse_tmp_0*flow_q[1] + 2*flow_q[2]*flow_q[3])*state[5] + (-cse_tmp_2 + cse_tmp_3 + cse_tmp_4)*state[4];
}

static void H_4(float *state, float *flow_q, float *out) {
  (void)state;
  const float cse_tmp_0 = (flow_q[1]) * (flow_q[1]);
  const float cse_tmp_1 = (flow_q[2]) * (flow_q[2]);
  const float cse_tmp_2 = (flow_q[0]) * (flow_q[0]) - (flow_q[3]) * (flow_q[3]);
  const float cse_tmp_3 = 2*flow_q[0];
  const float cse_tmp_4 = cse_tmp_3*flow_q[3];
  out[0] = 0;
  out[1] = 0;
  out[2] = 0;
  out[3] = cse_tmp_0 - cse_tmp_1 + cse_tmp_2;
  out[4] = cse_tmp_4 + 2*flow_q[1]*flow_q[2];
  out[5] = -cse_tmp_3*flow_q[2] + 2*flow_q[1]*flow_q[3];
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 0;
  out[11] = 0;
  out[12] = 0;
  out[13] = -cse_tmp_4 + 2*flow_q[1]*flow_q[2];
  out[14] = -cse_tmp_0 + cse_tmp_1 + cse_tmp_2;
  out[15] = cse_tmp_3*flow_q[1] + 2*flow_q[2]*flow_q[3];
  out[16] = 0;
  out[17] = 0;
  out[18] = 0;
  out[19] = 0;
}

static void h_5(float *state, float *unused, float *out) {
  (void)unused;
  out[0] = state[2];
}

static void H_5(float *state, float *unused, float *out) {
  (void)state;
  (void)unused;
  out[0] = 0;
  out[1] = 0;
  out[2] = 1;
  out[3] = 0;
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
}

static void h_6(float *state, float *unused, float *out) {
  (void)unused;
  out[0] = state[0];
  out[1] = state[1];
}

static void H_6(float *state, float *unused, float *out) {
  (void)state;
  (void)unused;
  out[0] = 1;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 0;
  out[11] = 1;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 0;
  out[16] = 0;
  out[17] = 0;
  out[18] = 0;
  out[19] = 0;
}

static void h_7(float *state, float *unused, float *out) {
  (void)unused;
  out[0] = state[3];
  out[1] = state[4];
}

static void H_7(float *state, float *unused, float *out) {
  (void)state;
  (void)unused;
  out[0] = 0;
  out[1] = 0;
  out[2] = 0;
  out[3] = 1;
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 0;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 1;
  out[15] = 0;
  out[16] = 0;
  out[17] = 0;
  out[18] = 0;
  out[19] = 0;
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


static inline void mat_mul(const float *a, int a_rows, int a_cols, const float *b, int b_cols, float *out) {
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


static inline void mat_mul_transpose_right(const float *a, int a_rows, int a_cols, const float *b, int b_rows, float *out) {
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


static inline void mat_mul_transpose_left(const float *a, int a_rows, int a_cols, const float *b, int b_cols, float *out) {
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


static inline void mat_transpose_vec_mul(const float *a, int rows, int cols, const float *x, float *out) {
  for (int i = 0; i < cols; ++i) {
    float sum = 0.0f;
    for (int j = 0; j < rows; ++j) {
      sum += a[j * cols + i] * x[j];
    }
    out[i] = sum;
  }
}


static inline float dot_product(const float *a, const float *b, int len) {
  float sum = 0.0f;
  for (int i = 0; i < len; ++i) {
    sum += a[i] * b[i];
  }
  return sum;
}


static inline int solve_linear_system_1x1(float *a, float *b, int nrhs) {
  const float pivot_eps = 1.0e-9f;
  const float pivot_abs = fabsf(a[0]);

  if (pivot_abs < pivot_eps) {
    return 0;
  }

  const float inv_pivot = 1.0f / a[0];
  a[0] = 1.0f;
  for (int j = 0; j < nrhs; ++j) {
    b[j] *= inv_pivot;
  }

  return 1;
}


static inline int solve_linear_system_2x2(float *a, float *b, int nrhs) {
  const float pivot_eps = 1.0e-9f;

  if (fabsf(a[2]) > fabsf(a[0])) {
    const float a00 = a[0];
    const float a01 = a[1];
    a[0] = a[2];
    a[1] = a[3];
    a[2] = a00;
    a[3] = a01;

    for (int j = 0; j < nrhs; ++j) {
      const float tmp = b[j];
      b[j] = b[nrhs + j];
      b[nrhs + j] = tmp;
    }
  }

  if (fabsf(a[0]) < pivot_eps) {
    return 0;
  }

  const float inv_a00 = 1.0f / a[0];
  a[0] = 1.0f;
  a[1] *= inv_a00;
  for (int j = 0; j < nrhs; ++j) {
    b[j] *= inv_a00;
  }

  const float factor10 = a[2];
  a[2] = 0.0f;
  a[3] -= factor10 * a[1];
  for (int j = 0; j < nrhs; ++j) {
    b[nrhs + j] -= factor10 * b[j];
  }

  if (fabsf(a[3]) < pivot_eps) {
    return 0;
  }

  const float inv_a11 = 1.0f / a[3];
  a[3] = 1.0f;
  for (int j = 0; j < nrhs; ++j) {
    b[nrhs + j] *= inv_a11;
  }

  const float factor01 = a[1];
  a[1] = 0.0f;
  for (int j = 0; j < nrhs; ++j) {
    b[j] -= factor01 * b[nrhs + j];
  }

  return 1;
}


static int solve_linear_system(int dim, float *a, float *b, int nrhs) {
  if (dim == 1) {
    return solve_linear_system_1x1(a, b, nrhs);
  }
  if (dim == 2) {
    return solve_linear_system_2x2(a, b, nrhs);
  }

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


static void kinematic_predict_covariance_sparse(float *in_P, float *in_Q, const float *in_F, float dt);

static void predict_covariance(float *in_P, float *in_Q, const float *in_F, float dt) {
  kinematic_predict_covariance_sparse(in_P, in_Q, in_F, dt);
}


static int __attribute__((unused)) update_core(int zdim, float *in_x, float *in_P, ObsFun h_fun, ObsFun H_fun, float *in_z, float *in_R,
                       float *in_ea, float maha_threshold, int do_maha) {
  float h[MAX_ZDIM];
  float H[MAX_ZDIM * DIM];
#ifndef H_MOD_IS_IDENTITY
  float H_mod[DIM * EDIM];
#endif
  float H_err[MAX_ZDIM * EDIM];
  float y[MAX_ZDIM];
  float R[MAX_ZDIM * MAX_ZDIM];
  float HP[MAX_ZDIM * EDIM];
  float S[MAX_ZDIM * MAX_ZDIM];
  float S_work[MAX_ZDIM * MAX_ZDIM];
  float KT[MAX_ZDIM * EDIM];
  float KH[EDIM * EDIM];
  float I_KH[EDIM * EDIM];
  float dx[EDIM];
  float x_new[DIM];
  float temp[EDIM * EDIM];
  float p_new[EDIM * EDIM];
  float RKT[MAX_ZDIM * EDIM];
  float KRKT[EDIM * EDIM];
  float maha_rhs[MAX_ZDIM];

  h_fun(in_x, in_ea, h);
  H_fun(in_x, in_ea, H);
#ifdef H_MOD_IS_IDENTITY
  memcpy(H_err, H, (size_t)(zdim * EDIM) * sizeof(float));
#else
  H_mod_fun(in_x, H_mod);
  mat_mul(H, zdim, DIM, H_mod, EDIM, H_err);
#endif

  for (int i = 0; i < zdim; ++i) {
    y[i] = in_z[i] - h[i];
    for (int j = 0; j < zdim; ++j) {
      R[i * zdim + j] = in_R[i * zdim + j];
    }
  }

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

/* kinematic_embedded_postprocess_v1 */
void kinematic_normalize_state(float *state);

static void kinematic_predict_covariance_sparse(float *in_P, float *in_Q, const float *in_F, float dt) {
#ifdef Q_SYM_IS_DISCRETE
  (void)dt;
#endif
  const float f03 = in_F[3];
  const float f07 = in_F[7];
  const float f08 = in_F[8];
  const float f09 = in_F[9];
  const float f14 = in_F[14];
  const float f17 = in_F[17];
  const float f18 = in_F[18];
  const float f19 = in_F[19];
  const float f25 = in_F[25];
  const float f27 = in_F[27];
  const float f28 = in_F[28];
  const float f29 = in_F[29];
  const float f37 = in_F[37];
  const float f38 = in_F[38];
  const float f39 = in_F[39];
  const float f47 = in_F[47];
  const float f48 = in_F[48];
  const float f49 = in_F[49];
  const float f57 = in_F[57];
  const float f58 = in_F[58];
  const float f59 = in_F[59];
  float p_next[EDIM * EDIM];

  for (int row = 0; row < EDIM; ++row) {
    float t[EDIM];
    for (int col = 0; col < EDIM; ++col) {
      const float p0 = in_P[col];
      const float p1 = in_P[EDIM + col];
      const float p2 = in_P[2 * EDIM + col];
      const float p3 = in_P[3 * EDIM + col];
      const float p4 = in_P[4 * EDIM + col];
      const float p5 = in_P[5 * EDIM + col];
      const float p6 = in_P[6 * EDIM + col];
      const float p7 = in_P[7 * EDIM + col];
      const float p8 = in_P[8 * EDIM + col];
      const float p9 = in_P[9 * EDIM + col];

      switch (row) {
        case 0:
          t[col] = p0 + f03 * p3 + f07 * p7 + f08 * p8 + f09 * p9;
          break;
        case 1:
          t[col] = p1 + f14 * p4 + f17 * p7 + f18 * p8 + f19 * p9;
          break;
        case 2:
          t[col] = p2 + f25 * p5 + f27 * p7 + f28 * p8 + f29 * p9;
          break;
        case 3:
          t[col] = p3 + f37 * p7 + f38 * p8 + f39 * p9;
          break;
        case 4:
          t[col] = p4 + f47 * p7 + f48 * p8 + f49 * p9;
          break;
        case 5:
          t[col] = p5 + f57 * p7 + f58 * p8 + f59 * p9;
          break;
        case 6:
          t[col] = p6;
          break;
        case 7:
          t[col] = p7;
          break;
        case 8:
          t[col] = p8;
          break;
        default:
          t[col] = p9;
          break;
      }
    }

    const int base = row * EDIM;
    p_next[base] = t[0] + f03 * t[3] + f07 * t[7] + f08 * t[8] + f09 * t[9];
    p_next[base + 1] = t[1] + f14 * t[4] + f17 * t[7] + f18 * t[8] + f19 * t[9];
    p_next[base + 2] = t[2] + f25 * t[5] + f27 * t[7] + f28 * t[8] + f29 * t[9];
    p_next[base + 3] = t[3] + f37 * t[7] + f38 * t[8] + f39 * t[9];
    p_next[base + 4] = t[4] + f47 * t[7] + f48 * t[8] + f49 * t[9];
    p_next[base + 5] = t[5] + f57 * t[7] + f58 * t[8] + f59 * t[9];
    p_next[base + 6] = t[6];
    p_next[base + 7] = t[7];
    p_next[base + 8] = t[8];
    p_next[base + 9] = t[9];
  }

#ifdef Q_SYM_IS_DISCRETE
  p_next[0] += in_Q[0];
  p_next[3] += in_Q[3];
  p_next[11] += in_Q[11];
  p_next[14] += in_Q[14];
  p_next[22] += in_Q[22];
  p_next[25] += in_Q[25];
  p_next[30] += in_Q[30];
  p_next[33] += in_Q[33];
  p_next[41] += in_Q[41];
  p_next[44] += in_Q[44];
  p_next[52] += in_Q[52];
  p_next[55] += in_Q[55];
  p_next[66] += in_Q[66];
  p_next[77] += in_Q[77];
  p_next[88] += in_Q[88];
  p_next[99] += in_Q[99];
#else
  p_next[0] += dt * in_Q[0];
  p_next[3] += dt * in_Q[3];
  p_next[11] += dt * in_Q[11];
  p_next[14] += dt * in_Q[14];
  p_next[22] += dt * in_Q[22];
  p_next[25] += dt * in_Q[25];
  p_next[30] += dt * in_Q[30];
  p_next[33] += dt * in_Q[33];
  p_next[41] += dt * in_Q[41];
  p_next[44] += dt * in_Q[44];
  p_next[52] += dt * in_Q[52];
  p_next[55] += dt * in_Q[55];
  p_next[66] += dt * in_Q[66];
  p_next[77] += dt * in_Q[77];
  p_next[88] += dt * in_Q[88];
  p_next[99] += dt * in_Q[99];
#endif

  memcpy(in_P, p_next, sizeof(p_next));
}


static void kinematic_update_scalar_sparse(float *in_x, float *in_P, float *in_z, float r, int primary_idx,
                                           int secondary_idx, int use_secondary, float maha_threshold, int do_maha) {
  const float pivot_eps = 1.0e-9f;
  float hp[EDIM];
  float k[EDIM];
  float p_new[EDIM * EDIM];
  const float predicted = in_x[primary_idx] + (use_secondary ? in_x[secondary_idx] : 0.0f);
  const float y = in_z[0] - predicted;

  for (int j = 0; j < EDIM; ++j) {
    hp[j] = in_P[primary_idx * EDIM + j];
    if (use_secondary) {
      hp[j] += in_P[secondary_idx * EDIM + j];
    }
  }

  float s = hp[primary_idx] + (use_secondary ? hp[secondary_idx] : 0.0f) + r;
  if (fabsf(s) < pivot_eps) {
    return;
  }

  if (do_maha && (y * y) / s > maha_threshold) {
    r *= 1.0e16f;
    s = hp[primary_idx] + (use_secondary ? hp[secondary_idx] : 0.0f) + r;
    if (fabsf(s) < pivot_eps) {
      return;
    }
  }

  const float inv_s = 1.0f / s;
  for (int i = 0; i < EDIM; ++i) {
    k[i] = hp[i] * inv_s;
  }

  for (int i = 0; i < EDIM; ++i) {
    const float ki = k[i];
    const int row_base = i * EDIM;
    float temp_row[EDIM];

    for (int j = 0; j < EDIM; ++j) {
      temp_row[j] = in_P[row_base + j] - ki * hp[j];
    }

    const float temp_h = temp_row[primary_idx] + (use_secondary ? temp_row[secondary_idx] : 0.0f);
    for (int j = 0; j < EDIM; ++j) {
      p_new[row_base + j] = temp_row[j] - temp_h * k[j] + r * ki * k[j];
    }
  }

  for (int i = 0; i < EDIM; ++i) {
    in_x[i] += k[i] * y;
  }
  kinematic_normalize_state(in_x);

  memcpy(in_P, p_new, sizeof(p_new));
  in_z[0] = y;
}


static void kinematic_update_flow_sparse(float *in_x, float *in_P, float *in_z, float *in_R, float *flow_q) {
  float h[2];
  float H[2 * EDIM];
  float hp0[EDIM];
  float hp1[EDIM];
  float s[4];
  float s_work[4];
  float kt[2 * EDIM];
  float p_new[EDIM * EDIM];
  float y[2];

  h_4(in_x, flow_q, h);
  H_4(in_x, flow_q, H);

  const float h00 = H[3];
  const float h01 = H[4];
  const float h02 = H[5];
  const float h10 = H[13];
  const float h11 = H[14];
  const float h12 = H[15];

  y[0] = in_z[0] - h[0];
  y[1] = in_z[1] - h[1];

  for (int j = 0; j < EDIM; ++j) {
    const float p3 = in_P[3 * EDIM + j];
    const float p4 = in_P[4 * EDIM + j];
    const float p5 = in_P[5 * EDIM + j];
    hp0[j] = h00 * p3 + h01 * p4 + h02 * p5;
    hp1[j] = h10 * p3 + h11 * p4 + h12 * p5;
    kt[j] = hp0[j];
    kt[EDIM + j] = hp1[j];
  }

  s[0] = hp0[3] * h00 + hp0[4] * h01 + hp0[5] * h02 + in_R[0];
  s[1] = hp0[3] * h10 + hp0[4] * h11 + hp0[5] * h12 + in_R[1];
  s[2] = hp1[3] * h00 + hp1[4] * h01 + hp1[5] * h02 + in_R[2];
  s[3] = hp1[3] * h10 + hp1[4] * h11 + hp1[5] * h12 + in_R[3];

  memcpy(s_work, s, sizeof(s_work));
  if (!solve_linear_system_2x2(s_work, kt, EDIM)) {
    return;
  }

  for (int i = 0; i < EDIM; ++i) {
    in_x[i] += kt[i] * y[0] + kt[EDIM + i] * y[1];
  }
  kinematic_normalize_state(in_x);

  for (int i = 0; i < EDIM; ++i) {
    const float ki0 = kt[i];
    const float ki1 = kt[EDIM + i];
    const int row_base = i * EDIM;
    float temp_row[EDIM];

    for (int j = 0; j < EDIM; ++j) {
      temp_row[j] = in_P[row_base + j] - ki0 * hp0[j] - ki1 * hp1[j];
    }

    const float temp_h0 = temp_row[3] * h00 + temp_row[4] * h01 + temp_row[5] * h02;
    const float temp_h1 = temp_row[3] * h10 + temp_row[4] * h11 + temp_row[5] * h12;

    for (int j = 0; j < EDIM; ++j) {
      const float kj0 = kt[j];
      const float kj1 = kt[EDIM + j];
      const float krkt = ki0 * (in_R[0] * kj0 + in_R[1] * kj1) + ki1 * (in_R[2] * kj0 + in_R[3] * kj1);
      p_new[row_base + j] = temp_row[j] - temp_h0 * kj0 - temp_h1 * kj1 + krkt;
    }
  }

  memcpy(in_P, p_new, sizeof(p_new));
  memcpy(in_z, y, sizeof(y));
}


static void kinematic_update_two_state_sparse(float *in_x, float *in_P, float *in_z, float *in_R,
                                              int primary_idx0, int primary_idx1,
                                              float maha_threshold, int do_maha) {
  float hp0[EDIM];
  float hp1[EDIM];
  float s[4];
  float s_work[4];
  float maha_rhs[2];
  float kt[2 * EDIM];
  float p_new[EDIM * EDIM];
  float y[2];
  float r00 = in_R[0];
  float r01 = in_R[1];
  float r10 = in_R[2];
  float r11 = in_R[3];

  y[0] = in_z[0] - in_x[primary_idx0];
  y[1] = in_z[1] - in_x[primary_idx1];

  for (int j = 0; j < EDIM; ++j) {
    hp0[j] = in_P[primary_idx0 * EDIM + j];
    hp1[j] = in_P[primary_idx1 * EDIM + j];
    kt[j] = hp0[j];
    kt[EDIM + j] = hp1[j];
  }

  s[0] = hp0[primary_idx0] + r00;
  s[1] = hp0[primary_idx1] + r01;
  s[2] = hp1[primary_idx0] + r10;
  s[3] = hp1[primary_idx1] + r11;

  if (do_maha) {
    memcpy(s_work, s, sizeof(s_work));
    memcpy(maha_rhs, y, sizeof(maha_rhs));
    if (!solve_linear_system_2x2(s_work, maha_rhs, 1)) {
      return;
    }
    if (y[0] * maha_rhs[0] + y[1] * maha_rhs[1] > maha_threshold) {
      r00 *= 1.0e16f;
      r01 *= 1.0e16f;
      r10 *= 1.0e16f;
      r11 *= 1.0e16f;
      s[0] = hp0[primary_idx0] + r00;
      s[1] = hp0[primary_idx1] + r01;
      s[2] = hp1[primary_idx0] + r10;
      s[3] = hp1[primary_idx1] + r11;
    }
  }

  memcpy(s_work, s, sizeof(s_work));
  if (!solve_linear_system_2x2(s_work, kt, EDIM)) {
    return;
  }

  for (int i = 0; i < EDIM; ++i) {
    in_x[i] += kt[i] * y[0] + kt[EDIM + i] * y[1];
  }
  kinematic_normalize_state(in_x);

  for (int i = 0; i < EDIM; ++i) {
    const float ki0 = kt[i];
    const float ki1 = kt[EDIM + i];
    const int row_base = i * EDIM;
    float temp_row[EDIM];

    for (int j = 0; j < EDIM; ++j) {
      temp_row[j] = in_P[row_base + j] - ki0 * hp0[j] - ki1 * hp1[j];
    }

    const float temp_h0 = temp_row[primary_idx0];
    const float temp_h1 = temp_row[primary_idx1];

    for (int j = 0; j < EDIM; ++j) {
      const float kj0 = kt[j];
      const float kj1 = kt[EDIM + j];
      const float krkt = ki0 * (r00 * kj0 + r01 * kj1) + ki1 * (r10 * kj0 + r11 * kj1);
      p_new[row_base + j] = temp_row[j] - temp_h0 * kj0 - temp_h1 * kj1 + krkt;
    }
  }

  memcpy(in_P, p_new, sizeof(p_new));
  memcpy(in_z, y, sizeof(y));
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

void kinematic_h_5(float *state, float *extra_args, float *out) {
  h_5(state, extra_args, out);
}

void kinematic_H_5(float *state, float *extra_args, float *out) {
  H_5(state, extra_args, out);
}

void kinematic_h_6(float *state, float *extra_args, float *out) {
  h_6(state, extra_args, out);
}

void kinematic_H_6(float *state, float *extra_args, float *out) {
  H_6(state, extra_args, out);
}

void kinematic_h_7(float *state, float *extra_args, float *out) {
  h_7(state, extra_args, out);
}

void kinematic_H_7(float *state, float *extra_args, float *out) {
  H_7(state, extra_args, out);
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
  (void)in_ea;
  kinematic_update_scalar_sparse(in_x, in_P, in_z, in_R[0], 2, 0, 0, MAHA_THRESH_2, 0);
}

void kinematic_update_3(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea) {
  (void)in_ea;
  kinematic_update_scalar_sparse(in_x, in_P, in_z, in_R[0], 2, 6, 1, MAHA_THRESH_3, 1);
}

void kinematic_update_4(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea) {
  (void)MAHA_THRESH_4;
  kinematic_update_flow_sparse(in_x, in_P, in_z, in_R, in_ea);
}

void kinematic_update_5(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea) {
  (void)in_ea;
  kinematic_update_scalar_sparse(in_x, in_P, in_z, in_R[0], 2, 0, 0, MAHA_THRESH_5, 1);
}

void kinematic_update_6(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea) {
  (void)in_ea;
  kinematic_update_two_state_sparse(in_x, in_P, in_z, in_R, 0, 1, MAHA_THRESH_6, 1);
}

void kinematic_update_7(float *in_x, float *in_P, float *in_z, float *in_R, float *in_ea) {
  (void)in_ea;
  kinematic_update_two_state_sparse(in_x, in_P, in_z, in_R, 3, 4, MAHA_THRESH_7, 1);
}
