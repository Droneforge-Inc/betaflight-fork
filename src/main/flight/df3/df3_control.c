#include "df3_control.h"
#include <math.h>
#include <string.h>

static const float gravity = 9.80665f;
static float clip(float v, float lo, float hi)
{
    return fmaxf(lo, fminf(hi, v));
}
void df3ControlReset(df3Control_t *c)
{
    memset(c, 0, sizeof(*c));
}

bool df3ControlConfigValid(const df3ControlConfig_t *p)
{
    if (!p || !isfinite(p->hoverThrottle) || p->hoverThrottle < .1f || p->hoverThrottle > .8f ||
        !isfinite(p->accelToThrottle) || p->accelToThrottle <= 0 || p->accelToThrottle > .1f ||
        !isfinite(p->maxTiltRad) || p->maxTiltRad < .05f || p->maxTiltRad > .7f) {
        return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(p->kp[i]) || p->kp[i] < 0 || p->kp[i] > 30 || !isfinite(p->kv[i]) || p->kv[i] < 0 ||
            p->kv[i] > 20 || !isfinite(p->ki[i]) || p->ki[i] < 0 || p->ki[i] > 10 || !isfinite(p->integralLimit[i]) ||
            p->integralLimit[i] < 0 || p->integralLimit[i] > 5) {
            return false;
        }
    }
    return true;
}

static bool finiteState(const df3Estimate_t *s)
{
    if (!s || !s->valid) {
        return false;
    }
    for (unsigned i = 0; i < DF3_NX; ++i) {
        if (!isfinite(s->x[i])) {
            return false;
        }
    }
    return true;
}

void df3ControlStep(df3Control_t *c, const df3ControlConfig_t *cfg, uint64_t now, bool selected, bool armed,
                    bool permit, bool nativeOverride, bool imuFresh, bool rangeFresh, bool flowFresh,
                    const df3Estimate_t *s, const df3Reference_t *reference, df3ControlOutput_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!selected || !armed || !permit || nativeOverride) {
        df3ControlReset(c);
        return;
    }
    out->mode = DF3_CONTROL_WAIT;
    if (!df3ControlConfigValid(cfg)) {
        return;
    }
    const float dt = c->lastUs && now > c->lastUs ? (float)(now - c->lastUs) * 1e-6f : DF3_CONTROL_INITIAL_DT_S;
    if (c->lastUs && (now <= c->lastUs || now - c->lastUs > DF3_CONTROL_MAX_GAP_US)) {
        c->fault = true;
    }
    c->lastUs = now;
    if (!imuFresh || !finiteState(s)) {
        if (c->engaged) {
            c->fault = true;
        } else {
            return;
        }
    }
    if (c->fault) {
        out->mode = DF3_CONTROL_FAULT;
        out->authority = true;
        return;
    }
    if (!c->engaged) {
        if (!reference || !reference->active || !rangeFresh || !flowFresh || !s->verticalReferenceValid) {
            return;
        }
        c->engaged = true;
    }
    out->authority = true;
    df3Reference_t target;
    if (reference && reference->active && rangeFresh && flowFresh) {
        target = *reference;
        c->lostReferenceUs = 0;
        out->mode = DF3_CONTROL_TRACK;
    } else {
        if (!c->lostReferenceUs) {
            c->lostReferenceUs = now;
            memset(&c->fallback, 0, sizeof(c->fallback));
            memcpy(c->fallback.position, s->x, 3 * sizeof(float));
            const float *q = s->x + DF3_Q;
            c->fallback.yaw = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
        }
        target = c->fallback;
        out->mode = DF3_CONTROL_HOLD;
        // Briefly brake/hold locally; prolonged reference loss initiates a
        // slow descent. Hardware failsafe and manual AUX deselection still win.
        if (now - c->lostReferenceUs > DF3_CONTROL_REFERENCE_HOLD_US || !flowFresh) {
            out->mode = DF3_CONTROL_LAND;
            target.velocity[2] = DF3_CONTROL_LAND_SPEED_M_S;
            c->fallback.position[2] += DF3_CONTROL_LAND_SPEED_M_S * dt;
            target.position[2] = c->fallback.position[2];
        }
        if (!rangeFresh) {
            c->fault = true;
            out->mode = DF3_CONTROL_FAULT;
            return;
        }
    }
    float candidate[3], raw[3], error[3];
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(target.position[i]) || !isfinite(target.velocity[i]) || !isfinite(target.acceleration[i])) {
            c->fault = true;
            out->mode = DF3_CONTROL_FAULT;
            return;
        }
        // Same terrain policy semantics as DF2: no Z position correction or
        // integral accumulation while the vertical reference is invalid.
        error[i] = (i == 2 && !s->verticalReferenceValid) ? 0 : target.position[i] - s->x[i];
        candidate[i] = clip(c->integral[i] + error[i] * dt, -cfg->integralLimit[i], cfg->integralLimit[i]);
        raw[i] = target.acceleration[i] + cfg->kp[i] * error[i] + cfg->kv[i] * (target.velocity[i] - s->x[DF3_V + i]) +
                 cfg->ki[i] * candidate[i];
    }
#ifdef USE_DF3_BLACKBOX
    // Observe the actual candidate used above, before antiwindup can reject it.
    for (unsigned i = 0; i < 3; ++i) {
        out->trace.axis[i] = (df3ControlAxisTrace_t){
            .reference = {target.position[i], target.velocity[i], target.acceleration[i]},
            .terms = {target.acceleration[i], cfg->kp[i] * error[i],
                      cfg->kv[i] * (target.velocity[i] - s->x[DF3_V + i]), cfg->ki[i] * candidate[i]},
            .requestedAccel = raw[i]};
    }
#endif
    raw[2] = clip(raw[2], -DF3_CONTROL_VERTICAL_ACCEL_LIMIT_M_S2, DF3_CONTROL_VERTICAL_ACCEL_LIMIT_M_S2);
    float horizontal = hypotf(raw[0], raw[1]);
    const float maxHorizontal = (gravity - raw[2]) * tanf(cfg->maxTiltRad);
    if (horizontal > maxHorizontal) {
        raw[0] *= maxHorizontal / horizontal;
        raw[1] *= maxHorizontal / horizontal;
    }
    if (!flowFresh) {
        raw[0] = raw[1] = 0;
        candidate[0] = c->integral[0];
        candidate[1] = c->integral[1];
    }
    const float *q = s->x + DF3_Q;
    const float yaw = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    const float cy = cosf(yaw), sy = sinf(yaw);
    const float forward = cy * raw[0] + sy * raw[1], right = -sy * raw[0] + cy * raw[1];
    const float vertical = gravity - raw[2];
    const float total = hypotf(hypotf(forward, right), vertical);
    const float throttle = cfg->hoverThrottle + cfg->accelToThrottle * (total - gravity);
    out->throttle = clip(throttle, DF3_CONTROL_MIN_THROTTLE, DF3_CONTROL_MAX_THROTTLE);
#ifdef USE_DF3_BLACKBOX
    out->trace.requestedThrottle = throttle;
    out->trace.yawReference = target.yaw;
#endif
    out->angleDeg[0] = atan2f(right, hypotf(forward, vertical)) * 57.295779513f;
    // BF's pitch angle is opposite the FRD quaternion pitch; positive BF
    // pitch tilts thrust forward. Roll has the same sign in both frames.
    out->angleDeg[1] = atan2f(forward, vertical) * 57.295779513f;
    // The native gyro/PID Z axis is opposite estimator FRD yaw.
    out->yawRateDeg = -clip(remainderf(target.yaw - yaw, 6.283185307f) * 2, -1, 1) * 57.295779513f;
    if (!isfinite(out->yawRateDeg) || !isfinite(out->angleDeg[0]) || !isfinite(out->angleDeg[1]) || !isfinite(total) ||
        !isfinite(throttle)) {
        c->fault = true;
        memset(out, 0, sizeof(*out));
        out->mode = DF3_CONTROL_FAULT;
        out->authority = true;
        return;
    }
    for (unsigned i = 0; i < 3; ++i) {
        const float requested = target.acceleration[i] + cfg->kp[i] * error[i] +
                                cfg->kv[i] * (target.velocity[i] - s->x[DF3_V + i]) + cfg->ki[i] * candidate[i];
        // Conditional integration: do not wind farther into acceleration or
        // collective saturation. Always allow integration out of saturation.
        bool allow = (requested - raw[i]) * error[i] <= 0;
        if (i == 2) {
            allow &= (throttle <= DF3_CONTROL_MAX_THROTTLE || error[i] > 0) &&
                     (throttle >= DF3_CONTROL_MIN_THROTTLE || error[i] < 0);
        }
        if (allow) {
            c->integral[i] = candidate[i];
        }
#ifdef USE_DF3_BLACKBOX
        out->trace.axis[i].integral = c->integral[i];
        out->trace.axis[i].integrationHeld = !allow;
#endif
        out->acceleration[i] = raw[i];
    }
}
