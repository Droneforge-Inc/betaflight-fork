/* Production: resumable optical-flow quantization correction.
 * Included by df3_flow.c alongside the sensor-window frontend. Split the
 * likelihood integration into bounded steps, then publish both axes' corrected
 * observations and variances together. A zero quantization step is a supported
 * bypass. The same implementation also serves retained research builds. */
void df3FlowQuantizedStart(df3FlowQuantizedJob_t *j, float step)
{
    memset(j, 0, sizeof(*j));
    j->step = step;
}
int df3FlowQuantizedResume(df3FlowQuantizedJob_t *j, const float prior[2], float z[3], const float h[3],
                           float H[3 * DF3_NE], float R[9])
{
    if (!j || !isfinite(j->step) || j->step < 0) {
        return -1;
    }
    if (j->step == 0) {
        return 1;
    }
    if (!prior || !z || !h || !H || !R) {
        return -1;
    }
    if (j->axis == 2) {
        for (unsigned axis = 0; axis < 2; ++axis) {
            z[axis] = j->observation[axis];
            R[4 * axis] = j->variance[axis];
            if (j->unobserved[axis]) {
                memset(H + axis * DF3_NE, 0, DF3_NE * sizeof(float));
            }
        }
        return 1;
    }
    const unsigned axis = j->axis;
    if (j->phase == 0) {
        j->p = fmaxf(0, prior[axis]);
        j->r = R[4 * axis];
        j->s = j->p + j->r;
        if (!isfinite(prior[axis]) || !isfinite(z[axis]) || !isfinite(h[axis]) || !isfinite(j->s) || j->r <= 0 ||
            !isfinite(j->r)) {
            return -1;
        }
        j->observation[axis] = z[axis];
        j->variance[axis] = j->r;
        const float widthSquared = j->step * j->step / j->s;
        if (!isfinite(widthSquared)) {
            return -1;
        }
        const float residual = z[axis] - h[axis], centerSquared = residual * residual / j->s;
        if (widthSquared <= .125f && centerSquared <= 9) {
            j->variance[axis] += j->step * j->step * (1.f / 12.f) * (1 + widthSquared * (1 - centerSquared) * .05f);
            j->observation[axis] -= residual * centerSquared * widthSquared * widthSquared * (1.f / 360.f);
            ++j->axis;
            return 0;
        }
        j->sd = sqrtf(j->s);
        j->center = (z[axis] - h[axis]) / j->sd;
        const float half = .5f * j->step / j->sd;
        j->lo = fmaxf(-8, j->center - half);
        const float hi = fminf(8, j->center + half);
        if (j->lo >= hi) {
            ++j->axis;
            return 0;
        }
        if (j->lo == -8 && hi == 8) {
            j->unobserved[axis] = true;
            j->observation[axis] = h[axis];
            ++j->axis;
            return 0;
        }
        j->peak = fmaxf(j->lo, fminf(0, hi));
        j->segments = (unsigned)ceilf(hi - j->lo);
        j->half = .5f * (hi - j->lo) / (float)j->segments;
        j->mass = j->first = j->second = 0;
        j->segment = 0;
        j->phase = 1;
        return 0;
    }
    if (j->phase == 1) {
        static const float nodes[5] = {-.9061798459f, -.5384693101f, 0, .5384693101f, .9061798459f};
        static const float weights[5] = {.2369268851f, .4786286705f, .5688888889f, .4786286705f, .2369268851f};
        const float mid = j->lo + (2.f * (float)j->segment + 1) * j->half;
        for (unsigned k = 0; k < 5; ++k) {
            const float y = mid + j->half * nodes[k], t = y - j->center;
            const float w = weights[k] * expf(-.5f * (y * y - j->peak * j->peak));
            j->mass += w;
            j->first += w * t;
            j->second += w * t * t;
        }
        if (++j->segment == j->segments) {
            j->phase = 2;
        }
        return 0;
    }
    if (!isfinite(j->mass) || j->mass <= 0) {
        return -1;
    }
    const float mean = j->first / j->mass;
    const float conditionalVariance = fmaxf(0, j->second / j->mass - mean * mean);
    const float information = 1 - fminf(1, conditionalVariance);
    if (information <= .00001f) {
        j->unobserved[axis] = true;
        j->observation[axis] = h[axis];
    } else {
        j->observation[axis] = h[axis] + j->sd * (j->center + mean) / information;
        j->variance[axis] = fmaxf(j->r, j->s / information - j->p);
    }
    if (!isfinite(j->observation[axis]) || !isfinite(j->variance[axis])) {
        return -1;
    }
    ++j->axis;
    j->phase = 0;
    return 0;
}
