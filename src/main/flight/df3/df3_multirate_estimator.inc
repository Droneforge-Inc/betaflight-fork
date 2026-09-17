/* Production: bounded acceleration-window observation phases. Included by
 * df3_resumable_estimator.inc inside df3_estimator.c to share the transaction
 * phases and failure handling. The frontend reducer and this model use
 * identical left-held sample support.
 * Two bounded passes: anchor raw rotations to the state at the window center,
 * then integrate the mean observation and its gyro-bias Jacobian.
 * The same implementation also supports the retained 100 Hz research build. */
static void multirateWindowFault(df3FusionJob_t *j, df3Estimator_t *e)
{
    ++j->historyFaults;
    fusionFail(j, e);
}
static void multirateWindowBegin(df3FusionJob_t *j, df3Estimator_t *e, const df3Event_t *event)
{
    float gyro[3], R[9];
    uint64_t next, left;
    if (event->us < DF3_EPOCH_US / 2 || !df3GyroHistoryIntervalHint(j->history, event->us - DF3_EPOCH_US / 2, event->us,
                                                                    false, gyro, &next, &left, &j->historyHint)) {
        multirateWindowFault(j, e);
        return;
    }
    j->windowStartUs = j->windowCursorUs = left;
    j->windowEndUs = event->us + DF3_EPOCH_US / 2;
    df3RotationReset(&j->windowRotation);
    memset(j->windowM, 0, sizeof(j->windowM));
    memset(j->windowBiasH, 0, sizeof(j->windowBiasH));
    df3QuaternionMatrix(e->core.x + DF3_Q, R);
    for (unsigned i = 0; i < 3; ++i) {
        j->windowForce[i] =
            R[i] * e->core.x[DF3_A] + R[3 + i] * e->core.x[DF3_A + 1] + R[6 + i] * (e->core.x[DF3_A + 2] - 9.80665f);
    }
    j->phase = FUSION_WINDOW_BACK;
}
static void multirateWindowBack(df3FusionJob_t *j, df3Estimator_t *e, const df3Event_t *event)
{
    float gyro[3];
    uint64_t next;
    if (!df3GyroHistoryIntervalHint(j->history, j->windowCursorUs, event->us, false, gyro, &next, NULL,
                                    &j->historyHint)) {
        multirateWindowFault(j, e);
        return;
    }
    df3Rotation_t *r = &j->windowRotation;
    df3RotationStep(r, gyro, e->core.x + DF3_BG, (float)(next - j->windowCursorUs) * 1e-6f, false);
    j->windowCursorUs = next;
    if (next == event->us) {
        float R[9], B[9];
        df3QuaternionMatrix(r->q, R);
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned k = 0; k < 3; ++k) {
                B[3 * i + k] = 0;
                for (unsigned l = 0; l < 3; ++l) {
                    B[3 * i + k] -= R[3 * i + l] * r->B[3 * l + k];
                }
            }
        }
        memcpy(r->A, R, sizeof(R));
        memcpy(r->B, B, sizeof(B));
        for (unsigned i = 1; i < 4; ++i) {
            r->q[i] = -r->q[i];
        }
        j->windowCursorUs = j->windowStartUs;
        j->phase = FUSION_WINDOW_FORWARD;
    }
}
static void multirateWindowForward(df3FusionJob_t *j, df3Estimator_t *e, const df3Event_t *event)
{
    float gyro[3], D[9], f[3];
    uint64_t next;
    if (!df3GyroHistoryIntervalHint(j->history, j->windowCursorUs, j->windowEndUs, false, gyro, &next, NULL,
                                    &j->historyHint)) {
        multirateWindowFault(j, e);
        return;
    }
    const uint64_t start = event->us - DF3_EPOCH_US / 2;
    const uint64_t lo = j->windowCursorUs > start ? j->windowCursorUs : start;
    const float weight = next > lo ? (float)(next - lo) / DF3_EPOCH_US : 0;
    df3Rotation_t *r = &j->windowRotation;
    df3QuaternionMatrix(r->q, D);
    for (unsigned i = 0; i < 3; ++i) {
        f[i] = 0;
        for (unsigned k = 0; k < 3; ++k) {
            j->windowM[3 * i + k] += weight * D[3 * k + i];
            f[i] += D[3 * k + i] * j->windowForce[k];
        }
    }
    const float K[9] = {0, -f[2], f[1], f[2], 0, -f[0], -f[1], f[0], 0};
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned k = 0; k < 3; ++k) {
            for (unsigned l = 0; l < 3; ++l) {
                j->windowBiasH[3 * i + k] += weight * K[3 * i + l] * r->B[3 * l + k];
            }
        }
    }
    df3RotationStep(r, gyro, e->core.x + DF3_BG, (float)(next - j->windowCursorUs) * 1e-6f, false);
    j->windowCursorUs = next;
    if (next == j->windowEndUs) {
        j->phase = FUSION_WINDOW_FINISH;
    }
}
