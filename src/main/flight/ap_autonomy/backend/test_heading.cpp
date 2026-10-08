#include "ap_ekf.h"
#include "runtime.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static uint64_t simulated_time;
uint64_t ap_runtime_time_us() { return simulated_time; }
void ap_runtime_panic(const char *reason)
{
    std::fprintf(stderr, "AP panic: %s\n", reason);
    std::abort();
}

static float yaw(const float q[4])
{
    return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

static float angle_error(float value, float expected)
{
    return std::atan2(std::sin(value - expected), std::cos(value - expected));
}

int main(int argc, char **argv)
{
    assert(argc == 3);
    const bool rotate = std::strcmp(argv[1], "rotate") == 0;
    const bool omit = std::strcmp(argv[1], "omit") == 0;
    const bool armed_guard = std::strcmp(argv[1], "armed-guard") == 0;
    assert(rotate || omit || armed_guard || std::strcmp(argv[1], "stationary") == 0);
    const float seed = std::strtof(argv[2], nullptr);
    constexpr float dt = .004f, pi = 3.14159265359f;
    assert(ap_ekf_init(250, 1));
    ap_ekf_input_t input{};
    ap_ekf_output_t output{};
    input.dt_s = dt;
    input.delta_velocity_mps[2] = -9.80665f * dt;
    input.range_m = .065f;
    input.sensor_position_body_m[2] = .035f;
    input.flow_quality = 255;
    input.heading_error_rad = .05f;
    float previous_rate = 0, max_error = 0, before_arm_yaw = 0;
    uint16_t reset_count = 0;
    bool observed_alignment = false;
    for (unsigned tick = 1; tick <= 12500; ++tick) {
        const float t = tick * dt;
        simulated_time = tick * 4000ull;
        const float u = std::clamp((t - 6) / 4, 0.0f, 1.0f);
        const float turn = rotate ? .7f * (u - std::sin(2 * pi * u) / (2 * pi)) : 0;
        const float rate = rotate ? .7f / 4 * (1 - std::cos(2 * pi * u)) : 0;
        input.time_us = simulated_time;
        input.armed = (armed_guard && t < 20) || t >= 41;
        input.delta_angle_rad[2] = .5f * (previous_rate + rate) * dt;
        previous_rate = rate;
        input.fresh = tick % 5 == 0 ? AP_EKF_FLOW | AP_EKF_RANGE : 0;
        if (!omit && tick % 5 == 0) {
            input.fresh |= AP_EKF_HEADING;
        }
        input.flow_time_ms = simulated_time / 1000 - 10;
        input.heading_time_ms = simulated_time / 1000;
        // A later BF heading report, including its arm-time reset, must never
        // become another independent yaw measurement after local alignment.
        input.heading_rad = observed_alignment ? -seed : seed;
        assert(ap_ekf_update(&input, &output));
        if (omit || (armed_guard && t < 20)) {
            assert(!output.heading_aligned);
            continue;
        }
        if (output.heading_aligned && !observed_alignment) {
            observed_alignment = true;
            reset_count = output.yaw_reset_count;
        }
        if (t > (armed_guard ? 30 : 15)) {
            constexpr uint32_t required = 1u | (1u << 2) | (1u << 3) | (1u << 5);
            assert(observed_alignment && output.healthy && output.heading_aligned && !output.faults);
            assert((output.filter_status & required) == required);
            assert(output.yaw_reset_count == reset_count);
            const float error = std::fabs(angle_error(yaw(output.quaternion), seed + turn));
            max_error = std::max(max_error, error);
            // Ground-motion detection has a finite threshold: its upstream
            // static-yaw assumption slightly truncates the smooth turn tails.
            assert(error < (rotate ? 4 * pi / 180 : .02f));
        }
        if (tick == 10249) {
            before_arm_yaw = yaw(output.quaternion);
        }
        if (tick == 10250) {
            assert(std::fabs(angle_error(yaw(output.quaternion), before_arm_yaw)) < .01f);
        }
    }
    assert(omit || observed_alignment);
    std::printf("Local heading %s seed %.1f deg: max error %.3f deg; later BF heading ignored\n",
                argv[1], seed * 180 / pi, max_error * 180 / pi);
    return 0;
}
