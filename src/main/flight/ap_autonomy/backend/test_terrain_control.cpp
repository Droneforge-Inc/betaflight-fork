#include "ap_control.h"
#include "runtime.h"

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

static ap_control_input hover_input()
{
    ap_control_input input{};
    input.dt_s = .004f;
    input.position_ned_m[2] = input.reference_position_ned_m[2] = -3.2f;
    input.quaternion_body_to_ned[0] = 1;
    input.specific_force_ned_mss[2] = -9.80665f;
    input.armed = input.achieved_thrust_valid = true;
    input.achieved_collective_thrust = .35f;
    return input;
}

static ap_control_output advance(ap_control_input &input, unsigned ticks)
{
    ap_control_output output{};
    for (unsigned i = 0; i < ticks; ++i) {
        simulated_time += 4000;
        input.time_us = simulated_time;
        assert(ap_control_step(&input, &output));
        assert(output.active && std::isfinite(output.collective_thrust));
        if (!(output.collective_thrust >= 0 && output.collective_thrust <= 1)) {
            std::printf("Invalid thrust %.5f at tick %u, terrain %u, reference Vd %.3f Ad %.3f, measured Vd %.3f\n",
                double(output.collective_thrust), i, unsigned(input.terrain_transition),
                double(input.reference_velocity_ned_ms[2]), double(input.reference_acceleration_ned_mss[2]),
                double(input.velocity_ned_ms[2]));
        }
        assert(output.collective_thrust >= 0 && output.collective_thrust <= 1);
    }
    return output;
}

static ap_control_output command(bool terrain, float velocity, float acceleration, float measured_velocity = 0)
{
    assert(ap_control_init(.35f));
    auto input = hover_input();
    // Establish the ordinary hover target before applying either paired command.
    advance(input, 100);
    input.terrain_transition = terrain;
    // The two cases differ in a large room-height position error, which the
    // transition case must suspend while retaining commanded vertical motion.
    if (terrain) {
        input.reference_position_ned_m[2] -= 2;
    }
    input.reference_velocity_ned_ms[2] = velocity;
    input.reference_acceleration_ned_mss[2] = acceleration;
    input.velocity_ned_ms[2] = measured_velocity;
    const auto output = advance(input, 500);
    ap_control_destroy();
    return output;
}

int main()
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    // Suspend the position term, not the stabilizing vertical controller.
    const auto hovering = command(true, 0, 0);
    assert(std::fabs(hovering.collective_thrust - .35f) < .001f);
    assert(std::fabs(hovering.velocity_target_ned_ms[2]) < .001f);
    // Reinitializing the placement-constructed controller must reseed the
    // acceleration integrator from achieved thrust on every armed transition.
    for (unsigned i = 0; i < 2; ++i) {
        const auto restarted = command(true, 0, 0);
        std::printf("Restarted hover %u: collective %.5f, acceleration integral %.5f\n",
            i, double(restarted.collective_thrust), double(restarted.vertical_accel_i));
        assert(std::fabs(restarted.collective_thrust - .35f) < .001f);
        assert(std::fabs(restarted.vertical_accel_i) < .001f);
    }
    const auto falling = command(true, 0, 0, .3f);
    std::printf("Hover %.5f; downward-drift response %.5f, target Vd %.5f Ad %.5f\n",
        double(hovering.collective_thrust), double(falling.collective_thrust),
        double(falling.velocity_target_ned_ms[2]), double(falling.acceleration_target_ned_mss[2]));
    assert(falling.collective_thrust > hovering.collective_thrust + .001f);
    assert(falling.acceleration_target_ned_mss[2] < -.1f);

    const float directions[] = {-1, 1};
    for (float direction : directions) {
        const auto normal = command(false, direction * .25f, direction * .2f);
        const auto terrain = command(true, direction * .25f, direction * .2f);
        std::printf("Command direction %.0f: ordinary/terrain thrust %.5f / %.5f, target Vd %.5f Ad %.5f\n",
            double(direction), double(normal.collective_thrust), double(terrain.collective_thrust),
            double(terrain.velocity_target_ned_ms[2]), double(terrain.acceleration_target_ned_mss[2]));
        assert(std::fabs(terrain.velocity_target_ned_ms[2] - normal.velocity_target_ned_ms[2]) < .0001f);
        assert(std::fabs(terrain.acceleration_target_ned_mss[2] - normal.acceleration_target_ned_mss[2]) < .0001f);
        assert(std::fabs(terrain.collective_thrust - normal.collective_thrust) < .0001f);
        assert(direction * terrain.velocity_target_ned_ms[2] > .1f);
        assert(direction * terrain.acceleration_target_ned_mss[2] > .1f);
        assert(direction * (terrain.collective_thrust - .35f) < -.001f);
    }

    // A mismatched hover seed creates a meaningful learning opportunity. Keep
    // that achieved thrust and level/zero-rate sensor state fixed while toggling
    // only ambiguity, so a frozen learner is distinguishable from no excitation.
    assert(ap_control_init(.35f));
    auto input = hover_input();
    input.terrain_transition = true;
    input.achieved_collective_thrust = .52f;
    input.reference_position_ned_m[2] -= 2;
    const auto ambiguous = advance(input, 1250);
    std::printf("Ambiguous hover: collective %.5f, learned hover %.5f\n",
        double(ambiguous.collective_thrust), double(ambiguous.hover_thrust));
    assert(ambiguous.hover_thrust == .35f);
    assert(std::fabs(ambiguous.collective_thrust - .52f) < .005f);
    input.terrain_transition = false;
    const auto resumed = advance(input, 100);
    std::printf("Resumed hover: collective %.5f, learned hover %.5f, target Vd %.5f\n",
        double(resumed.collective_thrust), double(resumed.hover_thrust), double(resumed.velocity_target_ned_ms[2]));
    assert(resumed.velocity_target_ned_ms[2] < -.1f);
    assert(resumed.collective_thrust > ambiguous.collective_thrust + .01f);
    assert(resumed.hover_thrust > ambiguous.hover_thrust);
    ap_control_destroy();
    std::printf("Terrain control: hover %.4f, drift correction %.4f, ambiguity thrust %.4f; "
        "resumed correction %.4f and learned hover %.4f. Velocity/acceleration commands preserved both ways.\n",
        double(hovering.collective_thrust), double(falling.collective_thrust),
        double(ambiguous.collective_thrust), double(resumed.collective_thrust), double(resumed.hover_thrust));
    return 0;
}
