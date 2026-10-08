#include "ap_ekf.h"

#include <AP_DAL/AP_DAL.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

static uint64_t simulated_time;
uint64_t ap_runtime_time_us() { return simulated_time; }
void ap_runtime_panic(const char *reason)
{
    std::fprintf(stderr, "AP panic: %s\n", reason);
    std::abort();
}

int main()
{
    assert(ap_ekf_init(250, 1));
    ap_ekf_input_t input{};
    ap_ekf_output_t output{};
    input.dt_s = .004f;
    input.delta_angle_dt_s = .00375f;
    input.delta_angle_rad[2] = .5f * input.delta_angle_dt_s;
    input.delta_velocity_mps[2] = -9.80665f * input.dt_s;
    input.time_us = simulated_time = 4000;
    assert(ap_ekf_update(&input, &output));

    // Check the contract consumed by EKF3, including each increment's own time.
    Vector3f angle, velocity;
    float angle_dt, velocity_dt;
    auto &ins = AP::dal().ins();
    assert(ins.get_delta_angle(0, angle, angle_dt));
    assert(ins.get_delta_velocity(0, velocity, velocity_dt));
    assert(angle_dt == input.delta_angle_dt_s && velocity_dt == input.dt_s);
    assert(fabsf(angle.z / angle_dt - .5f) < 1e-6f);
    assert(fabsf(velocity.z / velocity_dt + 9.80665f) < 1e-6f);
    assert(ins.get_loop_delta_t() == input.dt_s);

    input.time_us = simulated_time = 8000;
    const float invalid[] = {-.004f, .0501f,
        std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()};
    for (float dt : invalid) {
        input.delta_angle_dt_s = dt;
        assert(!ap_ekf_update(&input, &output));
        assert(ins.get_delta_angle(0, angle, angle_dt));
        assert(angle_dt == .00375f);
    }

    // Rejection must not consume the timestamp; the equal-rate API remains valid.
    input.delta_angle_dt_s = 0;
    input.delta_angle_rad[2] = .5f * input.dt_s;
    assert(ap_ekf_update(&input, &output));
    assert(ins.get_delta_angle(0, angle, angle_dt));
    assert(ins.get_delta_velocity(0, velocity, velocity_dt));
    assert(angle_dt == input.dt_s && velocity_dt == input.dt_s);
    assert(fabsf(angle.z / angle_dt - .5f) < 1e-6f);
    std::puts("Independent IMU intervals, equal-rate compatibility, and timing guards passed");
}
