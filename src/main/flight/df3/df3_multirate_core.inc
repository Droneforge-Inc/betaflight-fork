/* Production: gyro-history prediction, rotation/bias propagation and the
 * windowed observation model. Included by df3_resumable_core.inc inside
 * df3_core.c to share its private quaternion and process helpers.
 * Each prediction continuation consumes one measured gyro interval.
 * The same implementation also supports the retained 100 Hz research build. */
void df3RotationReset(df3Rotation_t *r)
{
    df3ZeroBytes(r, sizeof(*r));
    r->q[0] = 1;
    r->A[0] = r->A[4] = r->A[8] = 1;
}

static void multirateProduct(const float a[9], const float b[9], float c[9])
{
    /* Evaluate all nine entries before writing: output may alias either input.
     * Scalar temporaries avoid forcing a stack array through memcpy. */
    const float v0 = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
    const float v1 = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
    const float v2 = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];
    const float v3 = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
    const float v4 = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
    const float v5 = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];
    const float v6 = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
    const float v7 = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
    const float v8 = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
    c[0] = v0;
    c[1] = v1;
    c[2] = v2;
    c[3] = v3;
    c[4] = v4;
    c[5] = v5;
    c[6] = v6;
    c[7] = v7;
    c[8] = v8;
}

/* Same component arithmetic as df3QuaternionMatrix, stored transposed. */
static DF3_CCM_CODE void multirateQuaternionTranspose(const float q[4], float R[9])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0] = 1 - 2 * (y * y + z * z);
    R[3] = 2 * (x * y - w * z);
    R[6] = 2 * (x * z + w * y);
    R[1] = 2 * (x * y + w * z);
    R[4] = 1 - 2 * (x * x + z * z);
    R[7] = 2 * (y * z - w * x);
    R[2] = 2 * (x * z - w * y);
    R[5] = 2 * (y * z + w * x);
    R[8] = 1 - 2 * (x * x + y * y);
}

/* Constant row pairs let the compiler retain shared inputs in registers.
 * Keep the inner accumulation order and its initial +0 unchanged. */
static inline __attribute__((always_inline)) float multirateNoiseEntry(const float at[9], const float A[9],
                                                                       const float ac[9], const float C[9], float bb,
                                                                       unsigned i, unsigned j)
{
    float value = 0;
    for (unsigned k = 0; k < 3; ++k) {
        value += at[3 * i + k] * A[3 * j + k] + ac[3 * i + k] * C[3 * j + k] + C[3 * i + k] * ac[3 * j + k] +
                 bb * C[3 * i + k] * C[3 * j + k];
    }
    return value;
}

void df3RotationStep(df3Rotation_t *r, const float gyro[3], const float bias[3], float dt, bool noise)
{
    float theta[3], dq[4], A[9], C[9], v[9], K[9], K2[9];
    for (unsigned i = 0; i < 3; ++i) {
        theta[i] = (gyro[i] - bias[i]) * dt;
    }
    const float t2 = theta[0] * theta[0] + theta[1] * theta[1] + theta[2] * theta[2];
    skew(theta, K, K2);
    float jacobianB;
    if (t2 < .01f) {
        /* Raw IMU increments are normally well within 0.1 rad. Share the
         * existing analytic series, including cos(theta/2), instead of
         * evaluating sqrt/trigonometry and skew twice per raw interval.
         * The omitted cosine term is below 1e-15 throughout this branch;
         * accumulated quaternions still receive the ordinary normalization. */
        const float s = .5f + t2 * (-1.f / 48 + t2 * (1.f / 3840 - t2 * (1.f / 645120)));
        dq[0] = 1 + t2 * (-1.f / 8 + t2 * (1.f / 384 - t2 * (1.f / 46080)));
        for (unsigned i = 0; i < 3; ++i) {
            dq[i + 1] = s * theta[i];
        }
        const float jacobianA = .5f + t2 * (-1.f / 24 + t2 * (1.f / 720 - t2 * (1.f / 40320)));
        jacobianB = 1.f / 6 + t2 * (-1.f / 120 + t2 * (1.f / 5040 - t2 * (1.f / 362880)));
        for (unsigned k = 0; k < 9; ++k) {
            C[k] = (k % 4 == 0 ? 1.f : 0) - jacobianA * K[k] + jacobianB * K2[k];
        }
    } else {
        /* Preserve the large-increment path; no flight-rate assumption is
         * used to truncate a valid rotation. */
        expQuaternion(theta, dq);
        rightJacobian(theta, C);
        jacobianB = 0;
    }
    df3QuaternionMultiply(r->q, dq, r->q);
    normalize(r->q);
    multirateQuaternionTranspose(dq, A);
    for (unsigned k = 0; k < 9; ++k) {
        C[k] *= -dt;
    }
    /* The normalized accumulated quaternion gives the same transition. */
    multirateQuaternionTranspose(r->q, r->A);
    multirateProduct(A, r->B, v);
    for (unsigned k = 0; k < 9; ++k) {
        r->B[k] = v[k] + C[k];
    }
    if (!noise) {
        return;
    }

    /* [A C; 0 I] Q [A C; 0 I]^T, exploiting Qbb = scalar*I. */
    float at[9], ac[9], tt[9], tb[9];
    multirateProduct(A, r->T, at);
    multirateProduct(A, r->C, ac);
    tt[0] = multirateNoiseEntry(at, A, ac, C, r->bb, 0, 0);
    tt[1] = tt[3] = multirateNoiseEntry(at, A, ac, C, r->bb, 0, 1);
    tt[2] = tt[6] = multirateNoiseEntry(at, A, ac, C, r->bb, 0, 2);
    tt[4] = multirateNoiseEntry(at, A, ac, C, r->bb, 1, 1);
    tt[5] = tt[7] = multirateNoiseEntry(at, A, ac, C, r->bb, 1, 2);
    tt[8] = multirateNoiseEntry(at, A, ac, C, r->bb, 2, 2);
    for (unsigned k = 0; k < 9; ++k) {
        tb[k] = ac[k] + r->bb * C[k];
    }
    const float dt2 = dt * dt, dt3 = dt2 * dt;
    float a, b, d;
    if (t2 < .1f) {
        /* Explicit reciprocal constants avoid a hardware VDIV for each
         * coefficient under strict floating-point compiler flags. Horner
         * form retains the same analytic polynomial in fewer operations. */
        a = t2 < .01f ? jacobianB : 1.f / 6 + t2 * (-1.f / 120 + t2 * (1.f / 5040 - t2 * (1.f / 362880)));
        b = 1.f / 24 + t2 * (-1.f / 720 + t2 * (1.f / 40320 - t2 * (1.f / 3628800)));
        d = 1.f / 60 + t2 * (-1.f / 2520 + t2 * (1.f / 181440 - t2 * (1.f / 19958400)));
    } else {
        const float t = sqrtf(t2), st = sinf(t), ct = cosf(t);
        a = (t - st) / (t * t2);
        b = (t2 * .5f + ct - 1) / (t2 * t2);
        d = (t * t2 / 3 - 2 * t + 2 * st) / (t * t2 * t2);
    }
    for (unsigned k = 0; k < 9; ++k) {
        const float ident = k % 4 == 0 ? 1.f : 0;
        r->T[k] = tt[k] + .0004f * dt * ident + .000004f * dt3 * (ident * (1.f / 3) + d * K2[k]);
        r->C[k] = tb[k] - .000004f * dt2 * (ident * .5f - a * K[k] + b * K2[k]);
    }
    r->bb += .000004f * dt;
}

void df3CoreStartHistoryPredict(df3CoreJob_t *j, const df3GyroHistory_t *history, uint64_t start, uint64_t end)
{
    df3ZeroBytes(j, sizeof(*j));
    j->historyHint = UINT16_MAX;
    j->phase = 5;
    j->history = history;
    j->cursorUs = start;
    j->endUs = end;
    j->dt = (float)(end - start) * 1e-6f;
}

static bool df3MultiratePredictBegin(df3CoreJob_t *j, df3Core_t *c, df3Workspace_t *w)
{
    if (!j->history || j->endUs <= j->cursorUs || j->endUs - j->cursorUs > DF3_EPOCH_US) {
        return coreJobDone(j, DF3_INVALID_ARGUMENT);
    }
    if (!finiteArray(c->x, DF3_NX) || !finiteArray(c->P, N * N)) {
        return coreJobDone(j, DF3_NONFINITE);
    }
    /* Translation spans the whole segment; angular composition starts at I. */
    memcpy(j->nx, c->x, sizeof(j->nx));
    for (unsigned i = 0; i < 3; ++i) {
        j->nx[i] += c->x[DF3_V + i] * j->dt + c->x[DF3_A + i] * j->dt * j->dt * .5f;
        j->nx[DF3_V + i] += c->x[DF3_A + i] * j->dt;
        w->F[IX(i, DF3_EV + i)] = j->dt;
        w->F[IX(i, DF3_EA + i)] = j->dt * j->dt * .5f;
        w->F[IX(DF3_EV + i, DF3_EA + i)] = j->dt;
    }
    df3RotationReset(&j->rotation);
    j->phase = 6;
    return false;
}

static bool df3MultiratePredictStep(df3CoreJob_t *j, df3Core_t *c, df3Workspace_t *w)
{
    float gyro[3];
    uint64_t next;
    if (!df3GyroHistoryIntervalHint(j->history, j->cursorUs, j->endUs, false, gyro, &next, NULL, &j->historyHint)) {
        return coreJobDone(j, DF3_INVALID_ARGUMENT);
    }
    df3RotationStep(&j->rotation, gyro, c->x + DF3_BG, (float)(next - j->cursorUs) * 1e-6f, true);
    j->cursorUs = next;
    if (next == j->endUs) {
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned k = 0; k < 3; ++k) {
                w->F[IX(DF3_ET + i, DF3_ET + k)] = j->rotation.A[3 * i + k];
                w->F[IX(DF3_ET + i, DF3_EBG + k)] = j->rotation.B[3 * i + k];
            }
        }
        df3QuaternionMultiply(c->x + DF3_Q, j->rotation.q, j->nx + DF3_Q);
        j->phase = 2;
    }
    return false;
}

static void df3MultirateAddNoise(const df3CoreJob_t *j, float P[N * N])
{
    const float dt = j->dt, dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
    const float jerk[9] = {(9.f / 20) * dt5, (9.f / 8) * dt4, (9.f / 6) * dt3, (9.f / 8) * dt4, 3 * dt3,
                           (9.f / 2) * dt2,  (9.f / 6) * dt3, (9.f / 2) * dt2, 9 * dt};
    for (unsigned axis = 0; axis < 3; ++axis) {
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned k = 0; k < 3; ++k) {
                P[IX(3 * i + axis, 3 * k + axis)] += jerk[3 * i + k];
            }
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned k = 0; k < 3; ++k) {
            P[IX(DF3_ET + i, DF3_ET + k)] += j->rotation.T[3 * i + k];
            P[IX(DF3_ET + i, DF3_EBG + k)] += j->rotation.C[3 * i + k];
            P[IX(DF3_EBG + k, DF3_ET + i)] += j->rotation.C[3 * i + k];
        }
        P[IX(DF3_EBG + i, DF3_EBG + i)] += j->rotation.bb;
        P[IX(DF3_EBA + i, DF3_EBA + i)] += .000009f * dt;
    }
}

void df3WindowObservation(const float x[DF3_NX], const float M[9], const float biasH[9], float h[3], float H[3 * N])
{
    float f[3], R[9], K[9], unused[9];
    df3QuaternionMatrix(x + DF3_Q, R);
    for (unsigned i = 0; i < 3; ++i) {
        f[i] = R[i] * x[DF3_A] + R[3 + i] * x[DF3_A + 1] + R[6 + i] * (x[DF3_A + 2] - 9.80665f);
    }
    skew(f, K, unused);
    memset(H, 0, 3 * N * sizeof(float));
    for (unsigned i = 0; i < 3; ++i) {
        h[i] = x[DF3_BA + i];
        H[i * N + DF3_EBA + i] = 1;
        for (unsigned k = 0; k < 3; ++k) {
            h[i] += M[3 * i + k] * f[k];
        }
        for (unsigned k = 0; k < 3; ++k) {
            for (unsigned l = 0; l < 3; ++l) {
                H[i * N + DF3_EA + k] += M[3 * i + l] * R[3 * k + l];
                H[i * N + DF3_ET + k] += M[3 * i + l] * K[3 * l + k];
            }
            H[i * N + DF3_EBG + k] = biasH[3 * i + k];
        }
    }
}
