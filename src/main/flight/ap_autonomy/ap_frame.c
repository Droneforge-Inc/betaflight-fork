#include "ap_frame.h"
#ifdef USE_AP_AUTONOMY

#include <math.h>

void apFrameSet(apLocalFrame_t *frame, const float navPosition[3], float yawOffset, float clearance)
{
    *frame = (apLocalFrame_t){.origin = {navPosition[0], navPosition[1], navPosition[2] + clearance},
        .yawOffset = yawOffset, .sine = sinf(yawOffset), .cosine = cosf(yawOffset),
        .halfSine = sinf(.5f * yawOffset), .halfCosine = cosf(.5f * yawOffset)};
}

void apFrameVectorToLocal(const apLocalFrame_t *frame, const float nav[3], float local[3])
{
    const float x = frame->cosine * nav[0] + frame->sine * nav[1];
    const float y = -frame->sine * nav[0] + frame->cosine * nav[1];
    local[0] = x;
    local[1] = y;
    local[2] = nav[2];
}

void apFrameVectorToNav(const apLocalFrame_t *frame, const float local[3], float nav[3])
{
    const float x = frame->cosine * local[0] - frame->sine * local[1];
    const float y = frame->sine * local[0] + frame->cosine * local[1];
    nav[0] = x;
    nav[1] = y;
    nav[2] = local[2];
}

void apFramePositionToLocal(const apLocalFrame_t *frame, const float nav[3], float local[3])
{
    const float offset[3] = {nav[0] - frame->origin[0], nav[1] - frame->origin[1], nav[2] - frame->origin[2]};
    apFrameVectorToLocal(frame, offset, local);
}

void apFramePositionToNav(const apLocalFrame_t *frame, const float local[3], float nav[3])
{
    apFrameVectorToNav(frame, local, nav);
    for (unsigned i = 0; i < 3; ++i) {
        nav[i] += frame->origin[i];
    }
}

void apFrameQuaternionToLocal(const apLocalFrame_t *frame, const float nav[4], float local[4])
{
    // Left multiply by the inverse frame yaw; body axes stay FRD.
    const float c = frame->halfCosine, s = frame->halfSine;
    const float q[4] = {c * nav[0] + s * nav[3], c * nav[1] + s * nav[2],
        c * nav[2] - s * nav[1], c * nav[3] - s * nav[0]};
    for (unsigned i = 0; i < 4; ++i) {
        local[i] = q[i];
    }
}

float apFrameYawToNav(const apLocalFrame_t *frame, float localYaw)
{
    return remainderf(localYaw + frame->yawOffset, 6.28318530718f);
}

void apFrameBodyToNav(const float q[4], const float body[3], float nav[3])
{
    // q * v * conjugate(q), without constructing another rotation matrix.
    const float t[3] = {2 * (q[2] * body[2] - q[3] * body[1]),
        2 * (q[3] * body[0] - q[1] * body[2]),
        2 * (q[1] * body[1] - q[2] * body[0])};
    const float rotated[3] = {body[0] + q[0] * t[0] + q[2] * t[2] - q[3] * t[1],
        body[1] + q[0] * t[1] + q[3] * t[0] - q[1] * t[2],
        body[2] + q[0] * t[2] + q[1] * t[1] - q[2] * t[0]};
    for (unsigned i = 0; i < 3; ++i) {
        nav[i] = rotated[i];
    }
}
#endif
