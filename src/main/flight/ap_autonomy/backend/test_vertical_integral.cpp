#include "ap_control.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

static uint64_t simulated_time;
uint64_t ap_runtime_time_us() { return simulated_time; }
void ap_runtime_panic(const char *reason)
{
    std::fprintf(stderr, "AP panic: %s\n", reason);
    std::abort();
}

int main()
{
    constexpr float gravity = 9.80665f;
    assert(ap_control_init(.35f));
    ap_control_input input{};
    ap_control_output output{};
    input.dt_s = .004f;
    input.quaternion_body_to_ned[0] = 1;
    input.armed = input.achieved_thrust_valid = true;
    input.achieved_collective_thrust = .35f;
    input.specific_force_ned_mss[2] = -gravity;
    const auto advance = [&](unsigned ticks) {
        for (unsigned i = 0; i < ticks; ++i) {
            simulated_time += 4000;
            input.time_us = simulated_time;
            assert(ap_control_step(&input, &output));
            assert(output.active && std::isfinite(output.collective_thrust));
        }
    };
    advance(100);
    input.landed = true; // Isolate integral response from hover learning after takeover.
    input.specific_force_ned_mss[2] = -gravity + 1;
    advance(100); // Settle the acceleration-error filter before measuring slope.
    const float initial_i = output.vertical_accel_i;
    advance(250);
    const float change = output.vertical_accel_i - initial_i;
    std::printf("1 m/s^2 error for 1 s: integral change %.6f (expected -0.02)\n", double(change));
    assert(std::fabs(change + .02f) < .00001f);

    input.throttle_upper_limited = true;
    const float limited_i = output.vertical_accel_i;
    advance(250);
    std::printf("Limited integral %.6f -> %.6f, target acceleration %.6f\n", double(limited_i),
                double(output.vertical_accel_i), double(output.acceleration_target_ned_mss[2]));
    assert(std::fabs(output.vertical_accel_i - limited_i) < .000001f);
    input.specific_force_ned_mss[2] = -gravity - 1;
    advance(250);
    assert(output.vertical_accel_i > limited_i + .01f);
    assert(output.vertical_accel_i <= .0001f);
    std::puts("Vertical integral gain, saturation freeze and unwinding passed");
    ap_control_destroy();
}
