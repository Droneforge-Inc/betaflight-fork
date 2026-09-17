/* Droneforge DF3. Model provenance and parity tests: tools/df3/ in DF_Sim.
 * Float implementation of the existing 19/18-state Rednose ESEKF equations.
 * No allocation, Eigen, controller inputs, or simulator ground truth. */
#include "df3_core.h"
#include "df3_profile.h"
#ifdef USE_DF3_RESET_ASM
#include "df3_reset.h"
#endif

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

// G47X DF3 builds reserve executable CCM for these hot routines. Other
// targets keep their normal code placement; numerical compiler flags stay strict.
#ifdef USE_DF3_CCM
#define DF3_CCM_CODE __attribute__((section(".ccm_code.df3")))
#else
#define DF3_CCM_CODE
#endif

#define N DF3_NE
#define IX(r, c) ((r) * N + (c))

/* Local object-representation transfers. The G4 size-optimized libc moves
 * one byte per iteration even for our aligned covariance/job buffers.
 * may_alias permits integer transfers of float/struct bytes without changing
 * their effective type. Misaligned buffers and partial words use byte access.
 * Keep these loops out of line and prohibit lowering them back to libc. */
typedef uint32_t df3AliasWord_t __attribute__((may_alias));

__attribute__((noipa, optimize("O2", "no-tree-loop-distribute-patterns", "no-tree-vectorize", "no-if-conversion",
                               "no-if-conversion2"))) static DF3_CCM_CODE void
df3CopyBytes(void *destination, const void *source, size_t bytes)
{
    unsigned char *dst = destination;
    const unsigned char *src = source;
    if ((((uintptr_t)dst | (uintptr_t)src) & 3u) == 0) {
        df3AliasWord_t *d = (df3AliasWord_t *)dst;
        const df3AliasWord_t *s = (const df3AliasWord_t *)src;
        for (; bytes >= 16; bytes -= 16, d += 4, s += 4) {
            const uint32_t a = s[0], b = s[1], c = s[2], e = s[3];
            d[0] = a;
            d[1] = b;
            d[2] = c;
            d[3] = e;
        }
        for (; bytes >= 4; bytes -= 4) {
            *d++ = *s++;
        }
        dst = (unsigned char *)d;
        src = (const unsigned char *)s;
    }
    while (bytes--) {
        *dst++ = *src++;
    }
}

__attribute__((noipa, optimize("O2", "no-tree-loop-distribute-patterns", "no-tree-vectorize", "no-if-conversion",
                               "no-if-conversion2"))) static DF3_CCM_CODE void
df3ZeroBytes(void *destination, size_t bytes)
{
    unsigned char *dst = destination;
    if (((uintptr_t)dst & 3u) == 0) {
        df3AliasWord_t *d = (df3AliasWord_t *)dst;
        for (; bytes >= 16; bytes -= 16, d += 4) {
            d[0] = 0;
            d[1] = 0;
            d[2] = 0;
            d[3] = 0;
        }
        for (; bytes >= 4; bytes -= 4) {
            *d++ = 0;
        }
        dst = (unsigned char *)d;
    }
    while (bytes--) {
        *dst++ = 0;
    }
}

/* The experimental state machine is compiled at the end of this file so
 * it can reuse the exact release model primitives without exporting them. */

static DF3_CCM_CODE bool finiteArray(const float *a, unsigned n)
{
    // Both supported targets use IEEE-754 binary32. Classify its exponent
    // with integer instructions instead of transferring FPU comparison flags
    // for every covariance entry. memcpy preserves strict aliasing and keeps
    // signed zero, subnormals and all finite magnitudes valid.
    _Static_assert(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
                   "DF3 requires binary32 floats");
    unsigned i = 0;
    // Bound the four-value block explicitly; the tail never reads past n.
    for (; n - i >= 4; i += 4) {
        uint32_t b0, b1, b2, b3;
        memcpy(&b0, a + i, sizeof(b0));
        memcpy(&b1, a + i + 1, sizeof(b1));
        memcpy(&b2, a + i + 2, sizeof(b2));
        memcpy(&b3, a + i + 3, sizeof(b3));
        if ((b0 & UINT32_C(0x7f800000)) == UINT32_C(0x7f800000) ||
            (b1 & UINT32_C(0x7f800000)) == UINT32_C(0x7f800000) ||
            (b2 & UINT32_C(0x7f800000)) == UINT32_C(0x7f800000) ||
            (b3 & UINT32_C(0x7f800000)) == UINT32_C(0x7f800000)) {
            return false;
        }
    }
    for (; i < n; ++i) {
        uint32_t bits;
        memcpy(&bits, a + i, sizeof(bits));
        if ((bits & UINT32_C(0x7f800000)) == UINT32_C(0x7f800000)) {
            return false;
        }
    }
    return true;
}

static DF3_CCM_CODE bool normalize(float q[4])
{
    const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!isfinite(n) || n < 1e-9f) {
        return false;
    }
#ifdef USE_DF3_MULTIRATE
    /* One reciprocal replaces four serial VDIV instructions on Cortex-M4F.
     * Keep the same finite/small-norm guard; the normalized components differ
     * only by binary32 reciprocal rounding. */
    const float reciprocal = 1.f / n;
    for (unsigned i = 0; i < 4; ++i) {
        q[i] *= reciprocal;
    }
#else
    for (unsigned i = 0; i < 4; ++i) {
        q[i] /= n;
    }
#endif
    return true;
}

DF3_CCM_CODE void df3QuaternionMultiply(const float a[4], const float b[4], float out[4])
{
    const float r[4] = {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
    memcpy(out, r, sizeof(r));
}

DF3_CCM_CODE void df3QuaternionMatrix(const float q[4], float R[9])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0] = 1 - 2 * (y * y + z * z);
    R[1] = 2 * (x * y - w * z);
    R[2] = 2 * (x * z + w * y);
    R[3] = 2 * (x * y + w * z);
    R[4] = 1 - 2 * (x * x + z * z);
    R[5] = 2 * (y * z - w * x);
    R[6] = 2 * (x * z - w * y);
    R[7] = 2 * (y * z + w * x);
    R[8] = 1 - 2 * (x * x + y * y);
}

DF3_CCM_CODE void df3QuaternionRotate(const float q[4], const float v[3], float out[3])
{
    float R[9], r[3];
    df3QuaternionMatrix(q, R);
    for (unsigned i = 0; i < 3; ++i) {
        r[i] = R[3 * i] * v[0] + R[3 * i + 1] * v[1] + R[3 * i + 2] * v[2];
    }
    memcpy(out, r, sizeof(r));
}

static DF3_CCM_CODE void skew(const float v[3], float K[9], float K2[9])
{
    const float x = v[0], y = v[1], z = v[2];
    const float m[9] = {0, -z, y, z, 0, -x, -y, x, 0};
    memcpy(K, m, sizeof(m));
    const float norm2 = x * x + y * y + z * z;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            K2[3 * i + j] = v[i] * v[j] - (i == j ? norm2 : 0);
        }
    }
}

static DF3_CCM_CODE void expQuaternion(const float v[3], float q[4])
{
    const float t2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    const float t = sqrtf(t2);
    const float s = t2 < 0.01f ? 0.5f - t2 / 48.0f + t2 * t2 / 3840.0f - t2 * t2 * t2 / 645120.0f : sinf(t * 0.5f) / t;
    q[0] = cosf(t * 0.5f);
    for (unsigned i = 0; i < 3; ++i) {
        q[i + 1] = s * v[i];
    }
}

static DF3_CCM_CODE void logQuaternion(const float q[4], float v[3])
{
    const float s = q[0] < 0 ? -1.0f : 1.0f;
    const float n = sqrtf(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    const float scale = n < 1e-6f ? 2.0f : 2.0f * atan2f(n, s * q[0]) / n;
    for (unsigned i = 0; i < 3; ++i) {
        v[i] = s * scale * q[i + 1];
    }
}

static DF3_CCM_CODE void rightJacobian(const float v[3], float J[9])
{
    float K[9], K2[9];
    skew(v, K, K2);
    const float t2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2], t = sqrtf(t2);
    const float a = t2 < 0.01f ? 0.5f - t2 / 24 + t2 * t2 / 720 - t2 * t2 * t2 / 40320 : (1 - cosf(t)) / t2;
    const float b =
        t2 < 0.01f ? 1.0f / 6 - t2 / 120 + t2 * t2 / 5040 - t2 * t2 * t2 / 362880 : (t - sinf(t)) / (t2 * t);
    for (unsigned i = 0; i < 9; ++i) {
        J[i] = (i % 4 == 0 ? 1.0f : 0) - a * K[i] + b * K2[i];
    }
}

static DF3_CCM_CODE void inject(const float x[DF3_NX], const float dx[N], float out[DF3_NX])
{
    memcpy(out, x, sizeof(float) * DF3_NX);
    for (unsigned i = 0; i < 9; ++i) {
        out[i] += dx[i];
    }
    float dq[4];
    expQuaternion(dx + DF3_ET, dq);
    df3QuaternionMultiply(x + DF3_Q, dq, out + DF3_Q);
    for (unsigned i = 0; i < 6; ++i) {
        out[DF3_BA + i] += dx[DF3_EBA + i];
    }
}

#if defined(USE_DF3_MULTIRATE) && defined(USE_DF3_CCM)
/* Keep shared math in instruction RAM when the smaller fusion owner lets LTO
 * inline it into otherwise flash-resident callers. */
__attribute__((noinline))
#endif
DF3_CCM_CODE bool df3ProjectNominal(float x[DF3_NX], const float gyro[3], float dt)
{
    if (!isfinite(dt) || dt < 0 || !finiteArray(gyro, 3)) {
        return false;
    }
    float theta[3], dq[4];
    for (unsigned i = 0; i < 3; ++i) {
        x[i] += x[DF3_V + i] * dt + x[DF3_A + i] * dt * dt * .5f;
        x[DF3_V + i] += x[DF3_A + i] * dt;
        theta[i] = (gyro[i] - x[DF3_BG + i]) * dt;
    }
    expQuaternion(theta, dq);
    df3QuaternionMultiply(x + DF3_Q, dq, x + DF3_Q);
    return normalize(x + DF3_Q) && finiteArray(x, DF3_NX);
}

void df3CoreReset(df3Core_t *c)
{
    memset(c, 0, sizeof(*c));
    c->x[DF3_Q] = 1;
    const float rad = 0.01745329251994329577f;
    for (unsigned i = 0; i < N; ++i) {
        float p = i < 3     ? .25f
                  : i < 6   ? .49f
                  : i < 9   ? 4.0f
                  : i < 11  ? (8 * rad) * (8 * rad)
                  : i == 11 ? (20 * rad) * (20 * rad)
                  : i < 15  ? .36f
                            : (10 * rad) * (10 * rad);
        c->P[IX(i, i)] = p;
    }
}

bool df3CoreInitialize(df3Core_t *c, const float p[3], const float v[3], const float a[3], const float q[4])
{
    if (!c || !p || !v || !a || !q) {
        return false;
    }
    df3CoreReset(c);
    if (!finiteArray(p, 3) || !finiteArray(v, 3) || !finiteArray(a, 3)) {
        return false;
    }
    memcpy(c->x + DF3_P, p, 12);
    memcpy(c->x + DF3_V, v, 12);
    memcpy(c->x + DF3_A, a, 12);
    memcpy(c->x + DF3_Q, q, 16);
    if (!normalize(c->x + DF3_Q)) {
        df3CoreReset(c);
        return false;
    }
    return df3CoreHealthy(c);
}

DF3_CCM_CODE bool df3CoreHealthy(const df3Core_t *c)
{
    if (!c || !finiteArray(c->x, DF3_NX) || !finiteArray(c->P, N * N)) {
        return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (fabsf(c->x[DF3_P + i]) > 100 || fabsf(c->x[DF3_V + i]) > 20 || fabsf(c->x[DF3_A + i]) > 60 ||
            fabsf(c->x[DF3_BA + i]) > 5) {
            return false;
        }
    }
    const float *q = c->x + DF3_Q;
    if (fabsf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1) > 1e-3f) {
        return false;
    }
    for (unsigned i = 0; i < N; ++i) {
        if (c->P[IX(i, i)] < 0) {
            return false;
        }
    }
    return true;
}

static DF3_CCM_CODE void process(const float x[DF3_NX], const float gyro[3], float dt, float nx[DF3_NX], float F[N * N],
                                 bool materializeFullF)
{
    memcpy(nx, x, DF3_NX * sizeof(float));
    // The private sparse covariance kernel reads only the entries below.
    // Preserve complete F materialization for the public model interface.
    if (materializeFullF) {
        memset(F, 0, N * N * sizeof(float));
        for (unsigned i = 0; i < N; ++i) {
            F[IX(i, i)] = 1;
        }
    }
    float theta[3], dq[4], R[9], J[9];
    for (unsigned i = 0; i < 3; ++i) {
        nx[i] += x[DF3_V + i] * dt + x[DF3_A + i] * dt * dt * .5f;
        nx[DF3_V + i] += x[DF3_A + i] * dt;
        F[IX(i, DF3_EV + i)] = dt;
        F[IX(i, DF3_EA + i)] = dt * dt * .5f;
        F[IX(DF3_EV + i, DF3_EA + i)] = dt;
        theta[i] = (gyro[i] - x[DF3_BG + i]) * dt;
    }
    expQuaternion(theta, dq);
    df3QuaternionMultiply(x + DF3_Q, dq, nx + DF3_Q);
    df3QuaternionMatrix(dq, R);
    rightJacobian(theta, J);
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            F[IX(DF3_ET + i, DF3_ET + j)] = R[3 * j + i];
            F[IX(DF3_ET + i, DF3_EBG + j)] = -dt * J[3 * i + j];
        }
    }
}

static DF3_CCM_CODE void addProcessNoise(float P[N * N], const float x[DF3_NX], const float gyro[3], float dt)
{
    const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
    const float jerk[9] = {9 * dt5 / 20, 9 * dt4 / 8, 9 * dt3 / 6, 9 * dt4 / 8, 9 * dt3 / 3,
                           9 * dt2 / 2,  9 * dt3 / 6, 9 * dt2 / 2, 9 * dt};
    for (unsigned axis = 0; axis < 3; ++axis) {
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned j = 0; j < 3; ++j) {
                P[IX(3 * i + axis, 3 * j + axis)] += jerk[3 * i + j];
            }
        }
    }
    float v[3], K[9], K2[9];
    for (unsigned i = 0; i < 3; ++i) {
        v[i] = (gyro[i] - x[DF3_BG + i]) * dt;
    }
    skew(v, K, K2);
    const float t2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2], t = sqrtf(t2);
    /* Wider series domain avoids cancellation of theta^4/theta^5 terms in
     * single precision. These are the same analytic integrals as Rednose. */
    const float a = t2 < .1f ? 1.0f / 6 - t2 / 120 + t2 * t2 / 5040 - t2 * t2 * t2 / 362880 : (t - sinf(t)) / (t * t2);
    const float b = t2 < .1f ? 1.0f / 24 - t2 / 720 + t2 * t2 / 40320 - t2 * t2 * t2 / 3628800
                             : (t2 * .5f + cosf(t) - 1) / (t2 * t2);
    const float d = t2 < .1f ? 1.0f / 60 - t2 / 2520 + t2 * t2 / 181440 - t2 * t2 * t2 / 19958400
                             : (t * t2 / 3 - 2 * t + 2 * sinf(t)) / (t * t2 * t2);
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            const unsigned k = 3 * i + j;
            const float ident = i == j ? 1.0f : 0;
            P[IX(DF3_ET + i, DF3_ET + j)] += .0004f * dt * ident + .000004f * dt3 * (ident / 3 + d * K2[k]);
            const float cross = -.000004f * dt2 * (ident * .5f - a * K[k] + b * K2[k]);
            P[IX(DF3_ET + i, DF3_EBG + j)] += cross;
            P[IX(DF3_EBG + j, DF3_ET + i)] += cross;
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        P[IX(DF3_EBG + i, DF3_EBG + i)] += .000004f * dt;
        P[IX(DF3_EBA + i, DF3_EBA + i)] += .000009f * dt;
    }
}

void df3ModelPredict(const float x[DF3_NX], const float gyro[3], float dt, float nx[DF3_NX], float F[N * N],
                     float Q[N * N])
{
    process(x, gyro, dt, nx, F, true);
    memset(Q, 0, N * N * sizeof(float));
    addProcessNoise(Q, x, gyro, dt);
}

/* Prediction F has fixed block support: p <- (p,v,a), v <- (v,a),
 * theta <- (theta,bg), with identity a/ba/bg rows. Keep ascending source
 * index order and the original +0 initialization, including signed zero. */
static DF3_CCM_CODE void predictCovariance(const float P[N * N], const float F[N * N], float tmp[N * N],
                                           float out[N * N])
{
    // Pack each fixed support in its original ascending source order once.
    unsigned offsets[6];
    float coefficients[6];
    for (unsigned i = 0; i < N; ++i) {
        float *row = tmp + IX(i, 0);
        if (i >= DF3_ET && i < DF3_ET + 3) {
            unsigned count = 0;
            for (unsigned block = 0; block < 2; ++block) {
                for (unsigned axis = 0; axis < 3; ++axis) {
                    const unsigned k = (block ? DF3_EBG : DF3_ET) + axis;
                    const float a = F[IX(i, k)];
                    if (a != 0) {
                        coefficients[count] = a;
                        offsets[count++] = k * N;
                    }
                }
            }
#define PREDICT_ROW_TERM(k) sum += coefficients[k] * P[offsets[k] + j];
#define PREDICT_COLUMN_TERM(k) sum += tmp[IX(i, offsets[k])] * coefficients[k];
#define PREDICT_TERMS_1(term) term(0)
#define PREDICT_TERMS_2(term) PREDICT_TERMS_1(term) term(1)
#define PREDICT_TERMS_3(term) PREDICT_TERMS_2(term) term(2)
#define PREDICT_TERMS_4(term) PREDICT_TERMS_3(term) term(3)
#define PREDICT_TERMS_5(term) PREDICT_TERMS_4(term) term(4)
#define PREDICT_TERMS_6(term) PREDICT_TERMS_5(term) term(5)
#define PREDICT_ROW_CASE(terms)                                                                                        \
    case terms:                                                                                                        \
        for (unsigned j = 0; j < N; ++j) {                                                                             \
            float sum = 0;                                                                                             \
            PREDICT_TERMS_##terms(PREDICT_ROW_TERM) row[j] = sum;                                                      \
        }                                                                                                              \
        break
            switch (count) {
            case 0:
                memset(row, 0, N * sizeof(float));
                break;
                PREDICT_ROW_CASE(1);
                PREDICT_ROW_CASE(2);
                PREDICT_ROW_CASE(3);
                PREDICT_ROW_CASE(4);
                PREDICT_ROW_CASE(5);
                PREDICT_ROW_CASE(6);
            }
#undef PREDICT_ROW_CASE
        } else if (i < DF3_EV) {
            const float a = F[IX(i, i + 3)], b = F[IX(i, i + 6)];
            for (unsigned j = 0; j < N; ++j) {
                float sum = 0.0f + P[IX(i, j)];
                if (a != 0) {
                    sum += a * P[IX(i + 3, j)];
                }
                if (b != 0) {
                    sum += b * P[IX(i + 6, j)];
                }
                row[j] = sum;
            }
        } else if (i < DF3_EA) {
            const float a = F[IX(i, i + 3)];
            for (unsigned j = 0; j < N; ++j) {
                float sum = 0.0f + P[IX(i, j)];
                if (a != 0) {
                    sum += a * P[IX(i + 3, j)];
                }
                row[j] = sum;
            }
        } else {
            for (unsigned j = 0; j < N; ++j) {
                row[j] = 0.0f + P[IX(i, j)];
            }
        }
    }
    for (unsigned j = 0; j < N; ++j) {
        if (j >= DF3_ET && j < DF3_ET + 3) {
            unsigned count = 0;
            for (unsigned block = 0; block < 2; ++block) {
                for (unsigned axis = 0; axis < 3; ++axis) {
                    const unsigned k = (block ? DF3_EBG : DF3_ET) + axis;
                    const float a = F[IX(j, k)];
                    if (a != 0) {
                        coefficients[count] = a;
                        offsets[count++] = k;
                    }
                }
            }
#define PREDICT_COLUMN_CASE(terms)                                                                                     \
    case terms:                                                                                                        \
        for (unsigned i = j; i < N; ++i) {                                                                             \
            float sum = 0;                                                                                             \
            PREDICT_TERMS_##terms(PREDICT_COLUMN_TERM) out[IX(i, j)] = sum;                                            \
        }                                                                                                              \
        break
            switch (count) {
            case 0:
                for (unsigned i = j; i < N; ++i) {
                    out[IX(i, j)] = 0;
                }
                break;
                PREDICT_COLUMN_CASE(1);
                PREDICT_COLUMN_CASE(2);
                PREDICT_COLUMN_CASE(3);
                PREDICT_COLUMN_CASE(4);
                PREDICT_COLUMN_CASE(5);
                PREDICT_COLUMN_CASE(6);
            }
#undef PREDICT_COLUMN_CASE
#undef PREDICT_ROW_TERM
#undef PREDICT_COLUMN_TERM
#undef PREDICT_TERMS_1
#undef PREDICT_TERMS_2
#undef PREDICT_TERMS_3
#undef PREDICT_TERMS_4
#undef PREDICT_TERMS_5
#undef PREDICT_TERMS_6
        } else if (j < DF3_EV) {
            const float a = F[IX(j, j + 3)], b = F[IX(j, j + 6)];
            for (unsigned i = j; i < N; ++i) {
                float sum = 0.0f + tmp[IX(i, j)];
                if (a != 0) {
                    sum += tmp[IX(i, j + 3)] * a;
                }
                if (b != 0) {
                    sum += tmp[IX(i, j + 6)] * b;
                }
                out[IX(i, j)] = sum;
            }
        } else if (j < DF3_EA) {
            const float a = F[IX(j, j + 3)];
            for (unsigned i = j; i < N; ++i) {
                float sum = 0.0f + tmp[IX(i, j)];
                if (a != 0) {
                    sum += tmp[IX(i, j + 3)] * a;
                }
                out[IX(i, j)] = sum;
            }
        } else {
            for (unsigned i = j; i < N; ++i) {
                out[IX(i, j)] = 0.0f + tmp[IX(i, j)];
            }
        }
        for (unsigned i = j + 1; i < N; ++i) {
            out[IX(j, i)] = out[IX(i, j)];
        }
    }
}

// G is identity except for its attitude-error block J. The Joseph stage
// provides an exactly symmetric P, so only the three changed rows/columns
// need writing. Buffer the rows and the lower cross block before modifying
// P; preserve the old lower-triangle dot products and term order.
static DF3_CCM_CODE void resetCovariance(float P[N * N], const float J[9], float tmp[N * N])
{
#if defined(USE_DF3_MULTIRATE) && defined(USE_DF3_RESET_ASM)
    _Static_assert(N == 18 && DF3_ET == 9, "Reset assembly requires the fixed DF3 layout");
    df3ResetCovarianceM4f(P, J, tmp);
#elif defined(USE_DF3_MULTIRATE)
    /* Keep each dot-product in registers instead of rereading/replacing its
     * partial sum in memory after every coefficient. +0, zero skips, operand
     * order, and k=0,1,2 accumulation order match the original reset exactly.
     * tmp's original write footprint is retained for resumable-job parity. */
    for (unsigned r = 0; r < 3; ++r) {
        const float a = J[3 * r], b = J[3 * r + 1], c = J[3 * r + 2];
        float *row = tmp + IX(DF3_ET + r, 0);
        for (unsigned j = 0; j < N; ++j) {
            float sum = 0;
            if (a != 0) {
                sum += a * P[IX(DF3_ET, j)];
            }
            if (b != 0) {
                sum += b * P[IX(DF3_ET + 1, j)];
            }
            if (c != 0) {
                sum += c * P[IX(DF3_ET + 2, j)];
            }
            row[j] = sum;
        }
    }
    for (unsigned i = DF3_ET + 3; i < N; ++i) {
        for (unsigned k = 0; k < 3; ++k) {
            tmp[IX(i, DF3_ET + k)] = P[IX(i, DF3_ET + k)];
        }
    }
    for (unsigned j = 0; j < DF3_ET; ++j) {
        for (unsigned i = DF3_ET; i < DF3_ET + 3; ++i) {
            P[IX(j, i)] = P[IX(i, j)] = tmp[IX(i, j)];
        }
    }
    for (unsigned j = DF3_ET; j < DF3_ET + 3; ++j) {
        const float a = J[3 * (j - DF3_ET)], b = J[3 * (j - DF3_ET) + 1], c = J[3 * (j - DF3_ET) + 2];
        for (unsigned i = j; i < N; ++i) {
            float sum = 0;
            if (a != 0) {
                sum += tmp[IX(i, DF3_ET)] * a;
            }
            if (b != 0) {
                sum += tmp[IX(i, DF3_ET + 1)] * b;
            }
            if (c != 0) {
                sum += tmp[IX(i, DF3_ET + 2)] * c;
            }
            P[IX(i, j)] = sum;
            if (i != j) {
                P[IX(j, i)] = sum;
            }
        }
    }
#else

    for (unsigned r = 0; r < 3; ++r) {
        float *row = tmp + IX(DF3_ET + r, 0);
        memset(row, 0, N * sizeof(float));
        for (unsigned k = 0; k < 3; ++k) {
            const float a = J[3 * r + k];
            if (a == 0) {
                continue;
            }
            for (unsigned j = 0; j < N; ++j) {
                row[j] += a * P[IX(DF3_ET + k, j)];
            }
        }
    }
    for (unsigned i = DF3_ET + 3; i < N; ++i) {
        for (unsigned k = 0; k < 3; ++k) {
            tmp[IX(i, DF3_ET + k)] = P[IX(i, DF3_ET + k)];
        }
    }
    for (unsigned j = 0; j < DF3_ET; ++j) {
        for (unsigned i = DF3_ET; i < DF3_ET + 3; ++i) {
            P[IX(j, i)] = P[IX(i, j)] = tmp[IX(i, j)];
        }
    }
    for (unsigned j = DF3_ET; j < DF3_ET + 3; ++j) {
        for (unsigned i = j; i < N; ++i) {
            P[IX(i, j)] = 0;
        }
        for (unsigned k = 0; k < 3; ++k) {
            const float a = J[3 * (j - DF3_ET) + k];
            if (a == 0) {
                continue;
            }
            for (unsigned i = j; i < N; ++i) {
                P[IX(i, j)] += tmp[IX(i, DF3_ET + k)] * a;
            }
        }
        for (unsigned i = j + 1; i < N; ++i) {
            P[IX(j, i)] = P[IX(i, j)];
        }
    }
#endif
}

DF3_CCM_CODE df3Status_e df3CorePredict(df3Core_t *c, df3Workspace_t *w, const float gyro[3], float dt)
{
    if (!c || !w || !gyro || !isfinite(dt) || dt < 0) {
        return DF3_INVALID_ARGUMENT;
    }
    if (!finiteArray(gyro, 3) || !finiteArray(c->x, DF3_NX) || !finiteArray(c->P, N * N)) {
        return DF3_NONFINITE;
    }
    float nx[DF3_NX];
    {
        DF3_PROFILE_SCOPE(DF3_PROF_PREDICT_MODEL);
        process(c->x, gyro, dt, nx, w->F, false);
    }
    {
        DF3_PROFILE_SCOPE(DF3_PROF_PREDICT_COV);
        predictCovariance(c->P, w->F, w->tmp, w->next);
        addProcessNoise(w->next, c->x, gyro, dt);
    }
    if (!normalize(nx + DF3_Q)) {
        return DF3_INVALID_QUATERNION;
    }
    if (!finiteArray(nx, DF3_NX) || !finiteArray(w->next, N * N)) {
        return DF3_NONFINITE;
    }
    df3CopyBytes(c->x, nx, sizeof(nx));
    df3CopyBytes(c->P, w->next, sizeof(c->P));
    return DF3_OK;
}

#if defined(USE_DF3_MULTIRATE) && defined(USE_DF3_CCM)
/* Keep shared math in instruction RAM when the smaller fusion owner lets LTO
 * inline it into otherwise flash-resident callers. */
__attribute__((noinline))
#endif
DF3_CCM_CODE bool df3ModelObserve(const float x[DF3_NX], df3Observation_e kind, const float measuredQ[4], float h[3],
                                  float H[3 * N])
{
    df3ZeroBytes(H, 3 * N * sizeof(float));
    if (kind >= DF3_POSITION && kind <= DF3_ACCELERATION) {
        unsigned offset = 3 * (kind - 1);
        for (unsigned i = 0; i < 3; ++i) {
            h[i] = x[offset + i];
            H[i * N + offset + i] = 1;
        }
    } else if (kind == DF3_BODY_ACCELEROMETER) {
        float R[9], f[3], K[9], K2[9];
        df3QuaternionMatrix(x + DF3_Q, R);
        for (unsigned i = 0; i < 3; ++i) {
            f[i] = R[i] * x[DF3_A] + R[3 + i] * x[DF3_A + 1] + R[6 + i] * (x[DF3_A + 2] - 9.80665f);
            h[i] = f[i] + x[DF3_BA + i];
            H[i * N + DF3_EBA + i] = 1;
            for (unsigned j = 0; j < 3; ++j) {
                H[i * N + DF3_EA + j] = R[3 * j + i];
            }
        }
        skew(f, K, K2);
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned j = 0; j < 3; ++j) {
                H[i * N + DF3_ET + j] = K[3 * i + j];
            }
        }
    } else if (kind == DF3_ATTITUDE) {
        if (!measuredQ) {
            return false;
        }
        float qm[4];
        memcpy(qm, measuredQ, sizeof(qm));
        if (!normalize(qm)) {
            return false;
        }
        qm[1] = -qm[1];
        qm[2] = -qm[2];
        qm[3] = -qm[3];
        float relative[4], K[9], K2[9];
        df3QuaternionMultiply(qm, x + DF3_Q, relative);
        logQuaternion(relative, h);
        skew(h, K, K2);
        const float t2 = h[0] * h[0] + h[1] * h[1] + h[2] * h[2], t = sqrtf(t2);
        const float b = t2 < .01f ? 1.0f / 12 + t2 / 720 + t2 * t2 / 30240 : (1 - t * .5f / tanf(t * .5f)) / t2;
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned j = 0; j < 3; ++j) {
                H[i * N + DF3_ET + j] = (i == j ? 1.0f : 0) + .5f * K[3 * i + j] + b * K2[3 * i + j];
            }
        }
    } else {
        return false;
    }
    return finiteArray(h, 3) && finiteArray(H, 3 * N);
}

static DF3_CCM_CODE bool cholesky3(const float S[9], float L[9])
{
    memset(L, 0, 9 * sizeof(float));
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j <= i; ++j) {
            float s = S[3 * i + j];
            for (unsigned k = 0; k < j; ++k) {
                s -= L[3 * i + k] * L[3 * j + k];
            }
            if (i == j) {
                if (!(s > 0) || !isfinite(s)) {
                    return false;
                }
                L[3 * i + j] = sqrtf(s);
            } else {
                L[3 * i + j] = s / L[3 * j + j];
            }
        }
    }
    return true;
}

static DF3_CCM_CODE void solve3(const float L[9], const float rhs[3], float x[3])
{
    float y[3];
    for (int i = 0; i < 3; ++i) {
        float v = rhs[i];
        for (int j = 0; j < i; ++j) {
            v -= L[3 * i + j] * y[j];
        }
        y[i] = v / L[3 * i + i];
    }
    for (int i = 2; i >= 0; --i) {
        float v = y[i];
        for (int j = i + 1; j < 3; ++j) {
            v -= L[3 * j + i] * x[j];
        }
        x[i] = v / L[3 * i + i];
    }
}

DF3_CCM_CODE df3Status_e df3CoreUpdate(df3Core_t *c, df3Workspace_t *w, df3Observation_e kind, const float z[3],
                                       const float R[9], const float measuredQ[4], float threshold, df3Diagnostics_t *d)
{
    if (!c) {
        return DF3_INVALID_ARGUMENT;
    }
    float h[3], H[3 * N];
    if (!df3ModelObserve(c->x, kind, measuredQ, h, H)) {
        return DF3_INVALID_ARGUMENT;
    }
    return df3CoreUpdateObservation(c, w, z, R, h, H, threshold, d);
}

DF3_CCM_CODE df3Status_e df3CoreUpdateObservation(df3Core_t *c, df3Workspace_t *w, const float z[3], const float R[9],
                                                  const float h[3], const float H[3 * N], float threshold,
                                                  df3Diagnostics_t *d)
{
    if (d) {
        d->nis = NAN;
        d->residualNorm = NAN;
        d->correctionNorm = NAN;
    }
    if (!c || !w || !z || !R || !h || !H) {
        return DF3_INVALID_ARGUMENT;
    }
    if (!finiteArray(c->x, DF3_NX) || !finiteArray(c->P, N * N) || !finiteArray(z, 3) || !finiteArray(R, 9) ||
        !finiteArray(h, 3) || !finiteArray(H, 3 * N)) {
        return DF3_NONFINITE;
    }
    // A finite diagonal R is positive definite exactly when its diagonal
    // is positive. Its validation factor is discarded before factoring S.
    float L[9];
    if (R[1] == 0 && R[2] == 0 && R[3] == 0 && R[5] == 0 && R[6] == 0 && R[7] == 0) {
        if (!(R[0] > 0) || !(R[4] > 0) || !(R[8] > 0)) {
            return DF3_NOT_POSITIVE;
        }
    } else {
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned j = 0; j < i; ++j) {
                if (R[3 * i + j] != R[3 * j + i] &&
                    fabsf(R[3 * i + j] - R[3 * j + i]) >
                        1e-6f * fmaxf(1.0f, fmaxf(fabsf(R[3 * i + j]), fabsf(R[3 * j + i])))) {
                    return DF3_NOT_POSITIVE;
                }
            }
        }
        if (!cholesky3(R, L)) {
            return DF3_NOT_POSITIVE;
        }
    }
    float y[3], HP[3 * N], S[9], K[N * 3];
    // The built-in observations have mostly zero Jacobian entries. Record
    // their exact support once, also supporting fully dense custom H. Never
    // threshold small entries; retain ascending indices/summation order.
    uint8_t hColumns[3][N], hCount[3] = {0};
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned k = 0; k < N; ++k) {
            if (H[i * N + k] != 0) {
                hColumns[i][hCount[i]++] = k;
            }
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        y[i] = z[i] - h[i];
        float *const hp = HP + i * N;
        memset(hp, 0, N * sizeof(float));
        for (unsigned entry = 0; entry < hCount[i]; ++entry) {
            const unsigned k = hColumns[i][entry];
            const float coefficient = H[i * N + k];
            const float *const p = c->P + k * N;
            for (unsigned j = 0; j < N; ++j) {
                hp[j] += coefficient * p[j];
            }
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            float v = R[3 * i + j];
            for (unsigned entry = 0; entry < hCount[j]; ++entry) {
                const unsigned k = hColumns[j][entry];
                v += HP[i * N + k] * H[j * N + k];
            }
            S[3 * i + j] = v;
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < i; ++j) {
            S[3 * i + j] = S[3 * j + i] = .5f * (S[3 * i + j] + S[3 * j + i]);
        }
    }
    if (!cholesky3(S, L)) {
        return DF3_NOT_POSITIVE;
    }
    float sy[3];
    solve3(L, y, sy);
    const float nis = y[0] * sy[0] + y[1] * sy[1] + y[2] * sy[2];
    if (d) {
        d->nis = nis;
        d->residualNorm = sqrtf(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]);
    }
    if (!isfinite(nis) || nis < 0) {
        return DF3_NOT_POSITIVE;
    }
    if (isfinite(threshold) && threshold > 0 && nis > threshold) {
        return DF3_REJECTED;
    }
    // Expand the fixed 3x3 triangular solve so L stays in registers across
    // all 18 gain rows. Keep the original division and subtraction order.
    float dx[N], nx[DF3_NX];
    for (unsigned i = 0; i < N; ++i) {
        const float t0 = HP[i] / L[0];
        const float t1 = (HP[N + i] - L[3] * t0) / L[4];
        float v = HP[2 * N + i];
        v -= L[6] * t0;
        v -= L[7] * t1;
        const float t2 = v / L[8];
        const float k2 = t2 / L[8];
        const float k1 = (t1 - L[7] * k2) / L[4];
        v = t0;
        v -= L[3] * k1;
        v -= L[6] * k2;
        const float k0 = v / L[0];
        K[3 * i] = k0;
        K[3 * i + 1] = k1;
        K[3 * i + 2] = k2;
        float correction = 0;
        correction += k0 * y[0];
        correction += k1 * y[1];
        correction += k2 * y[2];
        dx[i] = correction;
    }
    if (d) {
        float n = 0;
        for (unsigned i = 0; i < N; ++i) {
            n += dx[i] * dx[i];
        }
        d->correctionNorm = sqrtf(n);
    }
    inject(c->x, dx, nx);
    {
        DF3_PROFILE_SCOPE(DF3_PROF_JOSEPH);
        /* Joseph update as two rank-three products, O(N^2), not dense N^3.
     * Keep each element's arithmetic order; specialize the fixed rank of 3. */
        for (unsigned i = 0; i < N; ++i) {
            const float k0 = K[3 * i], k1 = K[3 * i + 1], k2 = K[3 * i + 2];
            for (unsigned j = 0; j < N; ++j) {
                float v = c->P[IX(i, j)];
                v -= k0 * HP[j];
                v -= k1 * HP[N + j];
                v -= k2 * HP[2 * N + j];
                w->tmp[IX(i, j)] = v;
            }
        }
        // HP is dead after the left product; reuse it for B = KR - APH^T.
        // Compute every B row before writing the symmetric output matrix.
        float *const B = HP;
        for (unsigned i = 0; i < N; ++i) {
            for (unsigned k = 0; k < 3; ++k) {
                float aph = 0, kr = 0;
                for (unsigned entry = 0; entry < hCount[k]; ++entry) {
                    const unsigned l = hColumns[k][entry];
                    aph += w->tmp[IX(i, l)] * H[k * N + l];
                }
                for (unsigned l = 0; l < 3; ++l) {
                    kr += K[3 * i + l] * R[3 * l + k];
                }
                B[3 * i + k] = kr - aph;
            }
        }
        for (unsigned i = 0; i < N; ++i) {
            const float b0 = B[3 * i], b1 = B[3 * i + 1], b2 = B[3 * i + 2];
            float diagonal = w->tmp[IX(i, i)];
            diagonal += b0 * K[3 * i];
            diagonal += b1 * K[3 * i + 1];
            diagonal += b2 * K[3 * i + 2];
            w->next[IX(i, i)] = diagonal;
            for (unsigned j = 0; j < i; ++j) {
                float lower = w->tmp[IX(i, j)], upper = w->tmp[IX(j, i)];
                lower += b0 * K[3 * j];
                lower += b1 * K[3 * j + 1];
                lower += b2 * K[3 * j + 2];
                upper += B[3 * j] * K[3 * i];
                upper += B[3 * j + 1] * K[3 * i + 1];
                upper += B[3 * j + 2] * K[3 * i + 2];
                // Same lower + upper averaging as the original final pass.
                w->next[IX(i, j)] = w->next[IX(j, i)] = .5f * (lower + upper);
            }
        }
    }
    {
        DF3_PROFILE_SCOPE(DF3_PROF_RESET_COV);
        float J[9];
        rightJacobian(dx + DF3_ET, J);
        resetCovariance(w->next, J, w->tmp);
    }
    if (!normalize(nx + DF3_Q)) {
        return DF3_INVALID_QUATERNION;
    }
    if (!finiteArray(nx, DF3_NX) || !finiteArray(w->next, N * N)) {
        return DF3_NONFINITE;
    }
    df3CopyBytes(c->x, nx, sizeof(nx));
    df3CopyBytes(c->P, w->next, sizeof(c->P));
    return DF3_OK;
}

#ifdef USE_DF3_RESUMABLE
#include "df3_resumable_core.inc"
#endif
