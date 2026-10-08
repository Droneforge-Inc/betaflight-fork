#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AP_CONTROL_API __attribute__((visibility("default")))

/* Host SITL interface. NED metres, m/s, m/s²; body FRD radians/s.
 * Quaternion is [w,x,y,z], rotating body FRD into navigation NED.
 * Specific force includes the accelerometer's gravity reaction: at rest,
 * specific_force_ned_mss = [0,0,-9.80665]. Supply the separate INS 20 Hz
 * two-pole filtered acceleration, minus EKF body bias, rotated to NED; do not
 * derive this control feedback from raw EKF delta velocity / dt.
 * References share the EKF origin. */
typedef struct {
    uint64_t time_us;
    float dt_s;
    float position_ned_m[3];
    float velocity_ned_ms[3];
    float vertical_position_rate_mps;
    float quaternion_body_to_ned[4];
    float gyro_body_rads[3];
    float specific_force_ned_mss[3];
    float reference_position_ned_m[3];
    float reference_velocity_ned_ms[3];
    float reference_acceleration_ned_mss[3];
    float reference_yaw_rad;
    float achieved_collective_thrust; /* mean motor thrust after mixer limiting */
    uint16_t position_ne_reset_count;
    uint16_t position_d_reset_count;
    bool armed;
    bool landed;
    bool lateral_control_enabled; /* false: relax horizontal loops and command vertical thrust */
    bool throttle_lower_limited;
    bool throttle_upper_limited;
    bool attitude_limited;
    bool achieved_thrust_valid;
    bool vertical_degraded; /* use AP's complementary vertical rate / vibration control */
    bool terrain_transition; /* suspend room-height position correction during surface ambiguity */
} ap_control_input;

typedef struct {
    float body_rate_target_rads[3];
    float collective_thrust; /* 0..1 thrust domain, before motor nonlinearity */
    float hover_thrust;
    float attitude_target_rad[3];
    float position_target_ned_m[3];
    float velocity_target_ned_ms[3];
    float acceleration_target_ned_mss[3];
    /* Positive-down acceleration PID terms: their sum subtracts from hover. */
    float vertical_accel_p;
    float vertical_accel_i;
    float vertical_accel_d;
    float vertical_accel_ff;
    float vertical_velocity_mps; /* actual selected control feedback */
    bool vertical_degraded;
    bool active;
} ap_control_output;

/* One controller per firmware process. Defaults are upstream Copter gains.
 * hover_thrust is an initial estimate in normalized thrust, not PWM/duty. */
AP_CONTROL_API bool ap_control_init(float hover_thrust);
AP_CONTROL_API bool ap_control_step(const ap_control_input *input, ap_control_output *output);
AP_CONTROL_API void ap_control_destroy(void);

/* Upstream inverse thrust curve with expo 0.65, unity voltage compensation.
 * Apply to each mixed motor's normalized thrust. The caller owns idle/span,
 * arming and output clamping. Battery compensation is not implemented here. */
AP_CONTROL_API float ap_control_thrust_to_actuator(float thrust);
AP_CONTROL_API float ap_control_actuator_to_thrust(float actuator);

#ifdef __cplusplus
}
#endif
