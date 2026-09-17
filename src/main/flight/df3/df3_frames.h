/* Native Betaflight forward/left/up -> estimator forward/right/down.
 * A proper 180-degree rotation about X, for both polar and axial vectors.
 * Telemetry DTO axis labels are not the native sensor frame contract. */
#pragma once

static inline void df3NativeVectorToFrd(const float native[3], float scale, float frd[3])
{
    frd[0] = native[0] * scale;
    frd[1] = -native[1] * scale;
    frd[2] = -native[2] * scale;
}

static inline void df3NativeQuaternionToFrd(const float native[4], float frd[4])
{
    /* D R D, with D=diag(1,-1,-1), changes both body and local bases. */
    frd[0] = native[0];
    frd[1] = native[1];
    frd[2] = -native[2];
    frd[3] = -native[3];
}
