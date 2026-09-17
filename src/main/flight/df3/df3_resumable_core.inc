/* Production: estimator arithmetic split into bounded, resumable phases.
 * Included by df3_core.c to share its private matrix and process helpers.
 * Sparse products retain 1.3.6 per-element arithmetic order; jobs need no heap
 * or coroutine stack. Production enables USE_DF3_RESUMABLE and
 * USE_DF3_MULTIRATE together through df3_runtime.h. */
#include "df3_resumable.h"
#ifdef USE_DF3_JOSEPH_ASM
/* Production: native ARM kernels on Cortex-M4/M7 hard-float targets; portable
 * C branches below serve SITL and other architectures. Research builds also
 * use this ABI to execute the ARM kernels through the SITL emulator. */
#include "joseph.h"
#endif

#ifdef USE_DF3_CCM
/* Avoid duplicating/unrolling the already specialized row cases in scarce
 * instruction RAM. The source-level accumulation order is unchanged. */
__attribute__((optimize("Os")))
#endif
static DF3_CCM_CODE void predictCovarianceSlice(const float P[N * N], const float F[N * N], float tmp[N * N],
                                                float out[N * N], unsigned pass, unsigned first, unsigned end)
{
    // Pack each fixed support in its original ascending source order once.
    unsigned offsets[6];
    float coefficients[6];
    if (pass == 0) {
        for (unsigned i = first; i < end; ++i) {
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
    }
    if (pass == 1) {
        for (unsigned j = first; j < end; ++j) {
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
}

static bool coreJobDone(df3CoreJob_t *job, df3Status_e status)
{
    job->status = status;
    job->phase = 0;
    return true;
}
void df3CoreStartPredict(df3CoreJob_t *job, const float gyro[3], float dt)
{
    df3ZeroBytes(job, sizeof(*job));
    job->phase = 1;
    job->dt = dt;
    memcpy(job->gyro, gyro, sizeof(job->gyro));
}
void df3CoreStartUpdate(df3CoreJob_t *job, const float z[3], const float R[9], const float h[3], const float H[3 * N],
                        float threshold)
{
    df3ZeroBytes(job, sizeof(*job));
    job->phase = 10;
    job->threshold = threshold;
    memcpy(job->z, z, sizeof(job->z));
    memcpy(job->R, R, sizeof(job->R));
    memcpy(job->h, h, sizeof(job->h));
    df3CopyBytes(job->H, H, sizeof(job->H));
}
#ifdef USE_DF3_MULTIRATE
#include "df3_multirate_core.inc"
#endif
#ifdef USE_DF3_CCM
/* LTO otherwise inlines this into the flash-resident fusion dispatcher and
 * defeats the explicit CCM placement. Other targets retain normal inlining. */
DF3_CCM_CODE __attribute__((noinline))
#endif
bool df3CoreResume(df3CoreJob_t *job, df3Core_t *c, df3Workspace_t *w)
{
    if (!job || !c || !w) {
        return true;
    }
    /* Each timer ends on this return. Never count cooperative yield time as
     * arithmetic time, or compare a slice mean with an old whole-update mean. */
    DF3_PROFILE_SCOPE(DF3_PROF_CORE_SLICE);
    float *const nx = job->nx, *const dx = job->dx;
    float *const L = job->L, *const y = job->y, *const HP = job->HP, *const K = job->K;
#ifndef USE_DF3_JOSEPH_ASM
    float *const B = job->HP;
#endif
    const float *const z = job->z, *const R = job->R, *const h = job->h, *const H = job->H;
    uint8_t (*hColumns)[N] = job->columns, *const hCount = job->count;
    const float threshold = job->threshold;
    const unsigned end =
#ifdef USE_DF3_MULTIRATE
        /* Balance heterogeneous row work without changing entry/arithmetic order
         * or the three callbacks per pass. Other matrix passes remain six rows. */
        job->phase == 2    ? (job->row < DF3_PREDICT_LEFT_FIRST_END    ? DF3_PREDICT_LEFT_FIRST_END
                              : job->row < DF3_PREDICT_LEFT_SECOND_END ? DF3_PREDICT_LEFT_SECOND_END
                                                                       : N)
        : job->phase == 3  ? (job->row < DF3_PREDICT_RIGHT_FIRST_END    ? DF3_PREDICT_RIGHT_FIRST_END
                              : job->row < DF3_PREDICT_RIGHT_SECOND_END ? DF3_PREDICT_RIGHT_SECOND_END
                                                                        : N)
        : job->phase == 17 ? (job->row < DF3_SYM_FIRST_END    ? DF3_SYM_FIRST_END
                              : job->row < DF3_SYM_SECOND_END ? DF3_SYM_SECOND_END
                                                              : N)
        :
#endif
        job->row + DF3_SLICE_ROWS < N ? job->row + DF3_SLICE_ROWS
                                      : N;
    switch (job->phase) {
    case 0:
        return true;
    case 1: {
        DF3_PROFILE_SCOPE(DF3_PROF_PREDICT_MODEL);
        if (!isfinite(job->dt) || job->dt < 0) {
            return coreJobDone(job, DF3_INVALID_ARGUMENT);
        }
        if (!finiteArray(job->gyro, 3) || !finiteArray(c->x, DF3_NX) || !finiteArray(c->P, N * N)) {
            return coreJobDone(job, DF3_NONFINITE);
        }
        process(c->x, job->gyro, job->dt, nx, w->F, false);
        job->phase = 2;
        return false;
    }
    case 2:
    case 3: {
        DF3_PROFILE_SCOPE(DF3_PROF_PREDICT_COV);
        predictCovarianceSlice(c->P, w->F, w->tmp, w->next, job->phase - 2, job->row, end);
        if ((job->row = end) == N) {
            job->row = 0;
            ++job->phase;
#ifdef USE_DF3_MULTIRATE
            if (job->phase == 4 && job->history) {
                job->phase = 7;
            }
#endif
        }
        return false;
    }
    case 4:
        addProcessNoise(w->next, c->x, job->gyro, job->dt);
        job->phase = 20;
        return false;
#ifdef USE_DF3_MULTIRATE
    case 5:
        return df3MultiratePredictBegin(job, c, w);
    case 6:
        return df3MultiratePredictStep(job, c, w);
    case 7:
        df3MultirateAddNoise(job, w->next);
        job->phase = 20;
        return false;
#endif
    case 10: {
        if (!finiteArray(c->x, DF3_NX) || !finiteArray(c->P, N * N) || !finiteArray(z, 3) || !finiteArray(R, 9) ||
            !finiteArray(h, 3) || !finiteArray(H, 3 * N)) {
            return coreJobDone(job, DF3_NONFINITE);
        }
        // A finite diagonal R is positive definite exactly when its diagonal
        // is positive. Its validation factor is discarded before factoring S.

        if (R[1] == 0 && R[2] == 0 && R[3] == 0 && R[5] == 0 && R[6] == 0 && R[7] == 0) {
            if (!(R[0] > 0) || !(R[4] > 0) || !(R[8] > 0)) {
                return coreJobDone(job, DF3_NOT_POSITIVE);
            }
        } else {
            for (unsigned i = 0; i < 3; ++i) {
                for (unsigned j = 0; j < i; ++j) {
                    if (R[3 * i + j] != R[3 * j + i] &&
                        fabsf(R[3 * i + j] - R[3 * j + i]) >
                            1e-6f * fmaxf(1.0f, fmaxf(fabsf(R[3 * i + j]), fabsf(R[3 * j + i])))) {
                        return coreJobDone(job, DF3_NOT_POSITIVE);
                    }
                }
            }
            if (!cholesky3(R, L)) {
                return coreJobDone(job, DF3_NOT_POSITIVE);
            }
        }
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned k = 0; k < N; ++k) {
                if (H[i * N + k] != 0) {
                    hColumns[i][hCount[i]++] = k;
                }
            }
        }

        job->phase = 11;
        return false;
    }
    case 11: {
        for (unsigned i = job->row; i < job->row + 1; ++i) {
            y[i] = z[i] - h[i];
            float *const hp = HP + i * N;
            df3ZeroBytes(hp, N * sizeof(float));
            for (unsigned entry = 0; entry < hCount[i]; ++entry) {
                const unsigned k = hColumns[i][entry];
                const float coefficient = H[i * N + k];
                const float *const p = c->P + k * N;
                for (unsigned j = 0; j < N; ++j) {
                    hp[j] += coefficient * p[j];
                }
            }
        }

        if (++job->row == 3) {
            job->row = 0;
            job->phase = 12;
        }
        return false;
    }
    case 12: {
        float S[9];
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
            return coreJobDone(job, DF3_NOT_POSITIVE);
        }
        float sy[3];
        solve3(L, y, sy);
        const float nis = y[0] * sy[0] + y[1] * sy[1] + y[2] * sy[2];
        if (!isfinite(nis) || nis < 0) {
            return coreJobDone(job, DF3_NOT_POSITIVE);
        }
        if (isfinite(threshold) && threshold > 0 && nis > threshold) {
            return coreJobDone(job, DF3_REJECTED);
        }

        job->phase = 13;
        return false;
    }
    case 13: {
        DF3_PROFILE_SCOPE(DF3_PROF_GAIN_SLICE);
        for (unsigned i = job->row; i < end; ++i) {
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

        if ((job->row = end) == N) {
            job->row = 0;
            job->phase = 14;
        }
        return false;
    }
    case 14:
        inject(c->x, dx, nx);
        job->phase = 15;
        return false;
    case 15: {
        DF3_PROFILE_SCOPE(DF3_PROF_JOSEPH);
#ifdef USE_DF3_JOSEPH_ASM
        josephArgs args = {c->P, K, HP, H, R, (const uint8_t (*)[N])hColumns, hCount, w->tmp, w->next};
        joseph_asm_left(&args, job->row, end - job->row);
#else
        for (unsigned i = job->row; i < end; ++i) {
            const float k0 = K[3 * i], k1 = K[3 * i + 1], k2 = K[3 * i + 2];
            for (unsigned j = 0; j < N; ++j) {
                float v = c->P[IX(i, j)];
                v -= k0 * HP[j];
                v -= k1 * HP[N + j];
                v -= k2 * HP[2 * N + j];
                w->tmp[IX(i, j)] = v;
            }
        }
#endif

        if ((job->row = end) == N) {
            job->row = 0;
            job->phase = 16;
        }
        return false;
    }
    case 16: {
        DF3_PROFILE_SCOPE(DF3_PROF_JOSEPH);
#ifdef USE_DF3_JOSEPH_ASM
        josephArgs args = {c->P, K, HP, H, R, (const uint8_t (*)[N])hColumns, hCount, w->tmp, w->next};
        joseph_asm_middle(&args, job->row, end - job->row);
#else
        for (unsigned i = job->row; i < end; ++i) {
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
#endif

        if ((job->row = end) == N) {
            job->row = 0;
            job->phase = 17;
        }
        return false;
    }
    case 17: {
        DF3_PROFILE_SCOPE(DF3_PROF_JOSEPH);
#ifdef USE_DF3_JOSEPH_ASM
        josephArgs args = {c->P, K, HP, H, R, (const uint8_t (*)[N])hColumns, hCount, w->tmp, w->next};
        joseph_asm_sym(&args, job->row, end - job->row);
#else
        for (unsigned i = job->row; i < end; ++i) {
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
#endif

        if ((job->row = end) == N) {
            job->row = 0;
            job->phase = 18;
        }
        return false;
    }
    case 18: {
        DF3_PROFILE_SCOPE(DF3_PROF_RESET_COV);
        float J[9];
        rightJacobian(dx + DF3_ET, J);
        resetCovariance(w->next, J, w->tmp);
        job->phase = 20;
        return false;
    }
    case 20:
        if (!normalize(nx + DF3_Q)) {
            return coreJobDone(job, DF3_INVALID_QUATERNION);
        }
        if (!finiteArray(nx, DF3_NX) || !finiteArray(w->next, N * N)) {
            return coreJobDone(job, DF3_NONFINITE);
        }
        df3CopyBytes(c->x, nx, sizeof(c->x));
        df3CopyBytes(c->P, w->next, sizeof(c->P));
        return coreJobDone(job, DF3_OK);
    default:
        return coreJobDone(job, DF3_INVALID_ARGUMENT);
    }
}
