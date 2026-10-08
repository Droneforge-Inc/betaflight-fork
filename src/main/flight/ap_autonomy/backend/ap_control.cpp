#include "ap_control.h"
#include "runtime.h"

#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <cstdlib>
#include <cstring>
#include <new>


namespace {
constexpr float thrust_expo = 0.65f;
AP_AHRS_View &ahrs_state()
{
    static AP_AHRS_View value;
    return value;
}
AP_Scheduler scheduler;

class BetaflightAttitudeControl : public AC_AttitudeControl_Multi {
public:
    using AC_AttitudeControl_Multi::AC_AttitudeControl_Multi;
    // AP normally gets this snapshot from its rate loop. BF owns that loop.
    void set_gyro(const Vector3f &gyro) { _rate_gyro_rads = gyro; }
};

struct Controller {
    AP_MotorsMulticopter motors;
    BetaflightAttitudeControl attitude{ahrs_state(), motors};
    AC_PosControl position{ahrs_state(), motors, attitude};
    bool armed{};
};
Controller *controller;
alignas(Controller) unsigned char controller_storage[sizeof(Controller)];

Vector3f vector(const float v[3]) { return {v[0], v[1], v[2]}; }
template<typename T> void copy(float out[3], const Vector3<T> &v) {
    out[0] = v.x;
    out[1] = v.y;
    out[2] = v.z;
}
bool finite(const float *v, unsigned length) {
    for (unsigned i = 0; i < length; ++i) {
        if (!std::isfinite(v[i])) {
            return false;
        }
    }
    return true;
}
}

namespace AP {
AP_AHRS_View &ahrs() { return ahrs_state(); }
AP_Scheduler &scheduler() { return ::scheduler; }
}

bool ap_control_init(float hover_thrust)
{
    if (!std::isfinite(hover_thrust) || hover_thrust < 0.125f || hover_thrust > 0.6875f) {
        return false;
    }
    ap_backend_runtime_init();
    ap_control_destroy();
    // Upstream classes rely on ArduPilot's zero-initializing allocation.
    std::memset(controller_storage, 0, sizeof(controller_storage));
    auto &ahrs = ahrs_state();
    ahrs = {};
    scheduler = {};
    controller = new (controller_storage) Controller;
    controller->motors.hover = hover_thrust;
    // Reduce the Air75 II's measured acceleration-feedback ripple near 5 Hz.
    controller->position.D_get_accel_pid().set_kP(0.025f);
    // One fifth of Copter's 0.1 acceleration I gain for the indoor tuning trial.
    controller->position.D_get_accel_pid().set_kI(0.02f);
    controller->position.NE_set_max_speed_accel_m(5.0f, 1.0f);
    controller->position.NE_set_correction_speed_accel_m(5.0f, 1.0f);
    controller->position.D_set_max_speed_accel_m(1.5f, 2.5f, 2.5f);
    controller->position.D_set_correction_speed_accel_m(1.5f, 2.5f, 2.5f);
    return true;
}

void ap_control_destroy()
{
    if (controller) {
        controller->~Controller();
        controller = nullptr;
    }
}

bool ap_control_step(const ap_control_input *in, ap_control_output *out)
{
    if (!out) {
        return false;
    }
    *out = {};
    if (!controller || !in || !std::isfinite(in->dt_s) || in->dt_s <= 0 || in->dt_s > 0.05f ||
        !finite(in->position_ned_m, 3) || !finite(in->velocity_ned_ms, 3) ||
        !finite(in->quaternion_body_to_ned, 4) || !finite(in->gyro_body_rads, 3) ||
        !finite(in->specific_force_ned_mss, 3) || !finite(in->reference_position_ned_m, 3) ||
        !finite(in->reference_velocity_ned_ms, 3) || !finite(in->reference_acceleration_ned_mss, 3) ||
        (in->vertical_degraded && !std::isfinite(in->vertical_position_rate_mps)) ||
        !std::isfinite(in->reference_yaw_rad)) {
        return false;
    }
    ap_backend_set_time_us(in->time_us);
    Controller &c = *controller;
    auto &ahrs = ahrs_state();
    scheduler.dt = in->dt_s;
    ++scheduler.ticks;
    ahrs.position = vector(in->position_ned_m).topostype();
    ahrs.velocity = vector(in->velocity_ned_ms);
    ahrs.vertical_position_rate = in->vertical_position_rate_mps;
    ahrs.gyro = vector(in->gyro_body_rads);
    ahrs.specific_force = vector(in->specific_force_ned_mss);
    const float *q = in->quaternion_body_to_ned;
    ahrs.attitude = Quaternion(q[0], q[1], q[2], q[3]);
    if (ahrs.attitude.length_squared() < 0.5f || ahrs.attitude.length_squared() > 1.5f) {
        return false;
    }
    ahrs.attitude.normalize();
    ahrs.attitude.to_euler(ahrs.roll, ahrs.pitch, ahrs.yaw);
    ahrs.position_ne_reset_count = in->position_ne_reset_count;
    ahrs.position_d_reset_count = in->position_d_reset_count;
    c.motors.limit.throttle_lower = in->throttle_lower_limited;
    c.motors.limit.throttle_upper = in->throttle_upper_limited;
    c.motors.limit.roll = c.motors.limit.pitch = c.motors.limit.yaw = in->attitude_limited;
    c.attitude.set_dt_s(in->dt_s);
    c.attitude.set_gyro(ahrs.gyro);
    c.position.set_dt_s(in->dt_s);
    c.position.set_vibe_comp(in->vertical_degraded);
    c.position.update_estimates(in->vertical_degraded);
    out->vertical_velocity_mps = in->vertical_degraded ? ahrs.vertical_position_rate : ahrs.velocity.z;
    out->vertical_degraded = in->vertical_degraded;
    apAutonomyCheckpoint();

    if (!in->armed) {
        c.attitude.reset_target_and_rate();
        c.attitude.set_throttle_out(0, false, POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);
        c.motors.update_throttle_filter(in->dt_s, false);
        c.position.NE_init_controller();
        apAutonomyCheckpoint();
        c.position.D_init_controller();
        apAutonomyCheckpoint();
        c.armed = false;
        out->hover_thrust = c.motors.hover;
        return true;
    }
    if (!c.armed) {
        c.attitude.reset_target_and_rate();
        if (!in->landed && in->achieved_thrust_valid &&
            std::isfinite(in->achieved_collective_thrust) &&
            in->achieved_collective_thrust >= 0 && in->achieved_collective_thrust <= 1) {
            c.attitude.set_throttle_out(in->achieved_collective_thrust, false,
                                       POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);
            c.motors.throttle_filter.reset(in->achieved_collective_thrust);
        }
        c.position.NE_init_controller();
        apAutonomyCheckpoint();
        c.position.D_init_controller();
        apAutonomyCheckpoint();
        c.armed = true;
    }

    // Nimbus already supplies a time-sampled PVA trajectory. This upstream API
    // accepts external shaping without inserting a second trajectory generator.
    auto position_target = vector(in->reference_position_ned_m).topostype();
    if (in->terrain_transition) {
        // Terrain-only samples provide no independent room-height correction.
        // Preserve commanded velocity/acceleration and their stabilizing loops.
        position_target.z = ahrs.position.z;
    }
    c.position.set_pos_vel_accel_NED_m(position_target,
                                     vector(in->reference_velocity_ned_ms),
                                     vector(in->reference_acceleration_ned_mss));
    if (in->attitude_limited) {
        c.position.NE_set_externally_limited();
    }
    apAutonomyCheckpoint();
    if (in->lateral_control_enabled) {
        c.position.NE_update_controller();
        apAutonomyCheckpoint();
    } else {
        c.position.NE_relax_velocity_controller();
        apAutonomyCheckpoint();
    }
    c.position.D_update_controller();
    // The upstream override does not refresh PID diagnostics. Capture its
    // actual desired-acceleration contribution from the pre-angle-boost output.
    const float override_i = c.position.D_get_accel_pid().get_i();
    const float override_ff = c.motors.hover - c.attitude.get_throttle_in() - override_i;
    apAutonomyCheckpoint();
    Vector3f thrust_vector = c.position.get_thrust_vector();
    if (!in->lateral_control_enabled) {
        thrust_vector.x = thrust_vector.y = 0;
    }
    apAutonomyCheckpoint();
    c.attitude.input_thrust_vector_heading_rad(thrust_vector, in->reference_yaw_rad);
    apAutonomyCheckpoint();
    c.motors.update_throttle_filter(in->dt_s, true);

    // Copter::update_throttle_hover gating and AP_MotorsMulticopter's 10 s
    // learning law. Use achieved thrust when the BF mixer supplies it.
    const float hover_sample = in->achieved_thrust_valid ? in->achieved_collective_thrust : c.motors.get_throttle();
    if (!in->landed && !in->vertical_degraded && !in->terrain_transition &&
        !in->throttle_lower_limited && !in->throttle_upper_limited &&
        is_zero(in->reference_velocity_ned_ms[2]) && fabsf(ahrs.velocity.z) < 0.6f &&
        fabsf(ahrs.roll) < radians(5) && fabsf(ahrs.pitch) < radians(5) &&
        std::isfinite(hover_sample) && hover_sample > 0 && hover_sample <= 1) {
        c.motors.hover = constrain_float(c.motors.hover + in->dt_s / (in->dt_s + 10.0f) *
                                        (hover_sample - c.motors.hover), 0.125f, 0.6875f);
    }
    copy(out->body_rate_target_rads, c.attitude.rate_bf_targets());
    copy(out->attitude_target_rad, c.attitude.get_att_target_euler_rad());
    copy(out->position_target_ned_m, c.position.get_pos_target_NED_m());
    copy(out->velocity_target_ned_ms, c.position.get_vel_target_NED_ms());
    copy(out->acceleration_target_ned_mss, c.position.get_accel_target_NED_mss());
    if (!in->lateral_control_enabled) {
        out->acceleration_target_ned_mss[0] = out->acceleration_target_ned_mss[1] = 0;
    }
    const auto &pid = c.position.D_get_accel_pid().get_pid_info();
    out->vertical_accel_p = in->vertical_degraded ? 0 : pid.P;
    out->vertical_accel_i = in->vertical_degraded ? override_i : pid.I;
    out->vertical_accel_d = in->vertical_degraded ? 0 : pid.D;
    out->vertical_accel_ff = in->vertical_degraded ? override_ff : pid.FF;
    out->collective_thrust = c.motors.get_throttle();
    out->hover_thrust = c.motors.hover;
    out->active = true;
    return finite(out->body_rate_target_rads, 3) && std::isfinite(out->collective_thrust);
}

float ap_control_thrust_to_actuator(float thrust)
{
    if (!std::isfinite(thrust)) {
        return 0;
    }
    // AP_Motors_Thrust_Linearization::apply_thrust_curve_and_volt_scaling,
    // expo=0.65, lift_max=battery_scale=1. BF owns the idle/output span.
    thrust = constrain_float(thrust, 0, 1);
    return ((thrust_expo - 1.0f) + safe_sqrt((1.0f - thrust_expo) * (1.0f - thrust_expo) +
            4.0f * thrust_expo * thrust)) / (2.0f * thrust_expo);
}

float ap_control_actuator_to_thrust(float actuator)
{
    if (!std::isfinite(actuator)) {
        return 0;
    }
    actuator = constrain_float(actuator, 0, 1);
    return (1.0f - thrust_expo) * actuator + thrust_expo * actuator * actuator;
}
