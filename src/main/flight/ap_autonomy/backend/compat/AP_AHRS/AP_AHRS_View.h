#pragma once
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>

class AC_PosControl;

/* Read-only estimator snapshot consumed by the unmodified AP controllers. */
class AP_AHRS_View {
public:
    Vector3p position;
    Vector3f velocity, gyro, specific_force;
    float vertical_position_rate = 0;
    Quaternion attitude;
    float roll = 0, pitch = 0, yaw = 0;
    uint16_t position_ne_reset_count = 0, position_d_reset_count = 0;
    float cos_yaw() const { return cosf(yaw); }
    float sin_yaw() const { return sinf(yaw); }
    float cos_pitch() const { return cosf(pitch); }
    float cos_roll() const { return cosf(roll); }
    const Vector3f &get_gyro_latest() const { return gyro; }
    const Vector3f &get_accel_ef() const { return specific_force; }
    bool get_quat_body_to_ned(Quaternion &q) const { q = attitude; return true; }
    uint16_t get_position_NE_reset_count() const { return position_ne_reset_count; }
    uint16_t get_position_D_reset_count() const { return position_d_reset_count; }
    bool get_relative_position_NED_origin(Vector3p &p) const { p = position; return true; }
    bool get_relative_position_D_origin_float(float &d) const { d = position.z; return true; }
    bool get_velocity_NED(Vector3f &v) const { v = velocity; return true; }
    bool get_vert_pos_rate_D(float &v) const { v = vertical_position_rate; return true; }
    float get_control_gain_scaler_XY() const { return 1; }
    float get_control_gain_scaler_Z() const { return 1; }
};
namespace AP { AP_AHRS_View &ahrs(); }
