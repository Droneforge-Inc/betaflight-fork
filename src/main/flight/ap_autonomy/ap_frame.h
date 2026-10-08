#pragma once

/* SDK local forward/right/down frame relative to the EKF navigation frame.
 * Establish while disarmed and freeze before arming: Betaflight's arm-time
 * yaw reset must not rotate an outstanding reference or the reported state. */
typedef struct {
    float origin[3];
    float yawOffset, sine, cosine, halfSine, halfCosine;
} apLocalFrame_t;

void apFrameSet(apLocalFrame_t *frame, const float navPosition[3], float yawOffset, float clearance);
void apFrameVectorToLocal(const apLocalFrame_t *frame, const float nav[3], float local[3]);
void apFrameVectorToNav(const apLocalFrame_t *frame, const float local[3], float nav[3]);
void apFramePositionToLocal(const apLocalFrame_t *frame, const float nav[3], float local[3]);
void apFramePositionToNav(const apLocalFrame_t *frame, const float local[3], float nav[3]);
void apFrameQuaternionToLocal(const apLocalFrame_t *frame, const float nav[4], float local[4]);
float apFrameYawToNav(const apLocalFrame_t *frame, float localYaw);

/* Unit quaternion [w,x,y,z], body FRD to navigation NED. Supports aliasing. */
void apFrameBodyToNav(const float quaternion[4], const float body[3], float nav[3]);
