#include "ap_ekf.h"
#include "runtime.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

// Estimator mechanism regressions with coherent kinematics, not an aircraft
// dynamics model or a centimetre-level flight-accuracy qualification. Each
// scenario runs in a separate process because the backend owns one EKF instance.
static uint64_t simulated_time;
uint64_t ap_runtime_time_us() { return simulated_time; }
void ap_runtime_panic(const char *reason)
{
    std::fprintf(stderr, "AP panic: %s\n", reason);
    std::abort();
}

struct Motion {
    float position;
    float velocity;
};

static Motion transition(float time, float start, float duration, float distance)
{
    const float u = std::clamp((time - start) / duration, 0.0f, 1.0f);
    return {distance * u * u * u * (10 + u * (-15 + 6 * u)),
        distance / duration * 30 * u * u * (1 - u) * (1 - u)};
}

static bool named(const char *scenario, const char *name)
{
    return std::strcmp(scenario, name) == 0;
}

static Motion height(float time, const char *scenario)
{
    Motion result = transition(time, 42, 6, 3.2f);
    if (named(scenario, "maneuver") || named(scenario, "maneuver-terrain")) {
        const Motion up = transition(time, 62, 5, 1.0f);
        const Motion down = transition(time, 74, 6, -1.6f);
        const Motion back = transition(time, 88, 5, .6f);
        result.position += up.position + down.position + back.position;
        result.velocity += up.velocity + down.velocity + back.velocity;
    }
    return result;
}

static Motion north(float time)
{
    return transition(time, 50, 58, 18);
}

static float terrain(float time, const char *scenario)
{
    if (named(scenario, "steps")) {
        // Revisit the same floor and objects: offsets must not ratchet on each
        // edge, and the second drop is the full 2 m discontinuity.
        if (time >= 56 && time < 61) { return .4f; }
        if (time >= 61 && time < 66) { return .8f; }
        if (time >= 66 && time < 73) { return 2; }
        if (time >= 80 && time < 86) { return 2; }
        if (time >= 92 && time < 98) { return .4f; }
    } else if (named(scenario, "slope")) {
        // An isolated long ramp followed by a plateau. Room altitude becomes
        // unobservable after the fixed coast allowance; the test must expose
        // that limit instead of demanding indefinite inertial accuracy.
        return transition(time, 56, 18, 1.4f).position;
    } else if (named(scenario, "gap-step") && time >= 60.5f) {
        return .8f;
    } else if (named(scenario, "maneuver-terrain") && time >= 56 && time < 99) {
        return .8f;
    }
    return 0;
}

int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    assert(argc == 2);
    const char *scenario = argv[1];
    assert(named(scenario, "steps") || named(scenario, "slope") || named(scenario, "maneuver") ||
        named(scenario, "maneuver-terrain") || named(scenario, "motor-bias") ||
        named(scenario, "stale") || named(scenario, "invalid") || named(scenario, "gap-step"));
    assert(ap_ekf_init(250, 1));
    constexpr float dt = .004f, gravity = 9.80665f, launch_range = .065f;
    ap_ekf_input_t input{};
    ap_ekf_output_t state{};
    input.dt_s = dt;
    input.sensor_position_body_m[2] = .035f;
    input.flow_quality = 255;
    input.heading_error_rad = .05f;
    float initial_down = 0, initial_north = 0, maximum_height_error = 0, maximum_velocity_error = 0;
    float maximum_north_error = 0, maximum_north_velocity_error = 0;
    float error_time = 0, final_height_error = 0;
    uint16_t reset_count = 0;
    uint32_t available_memory = 0, maximum_height_age = 0;
    bool observed_stale_height = false;
    bool saw_transition = false, saw_expiry = false, saw_reacquisition = false;
    bool previous_transition = false;
    uint32_t previous_transition_age = 0, previous_height_age = 0;
    float first_transition_s = 0, first_expiry_s = 0, maximum_pre_expiry_error = 0;
    float previous_down = 0, maximum_down_jump = 0;
    float initial_terrain = 0;
    bool gap_height_reacquired = false;
    unsigned unexpected_resets = 0;
    for (unsigned tick = 1; tick <= 28000; ++tick) {
        const float time = tick * dt;
        const Motion z = height(time, scenario), previous_z = height(time - dt, scenario);
        const Motion x = north(time), previous_x = north(time - dt);
        simulated_time = uint64_t(tick) * 4000;
        input.time_us = simulated_time;
        input.armed = time >= 41;
        input.automatic_throttle = true;
        input.takeoff_expected = time >= 41 && time < 49;
        input.delta_velocity_mps[0] = x.velocity - previous_x.velocity;
        input.delta_velocity_mps[2] = -(z.velocity - previous_z.velocity) - gravity * dt;
        if (named(scenario, "motor-bias") && time >= 56 && time < 74) {
            // Measurement corruption only. The real aircraft remains level;
            // a bias-driven inertial disagreement must not redefine the floor.
            input.delta_velocity_mps[2] += dt;
        }
        input.range_m = launch_range + z.position - terrain(time, scenario);
        input.fresh = 0;
        if (tick % 5 == 0) {
            input.fresh = AP_EKF_RANGE | AP_EKF_FLOW | AP_EKF_HEADING;
            input.heading_time_ms = simulated_time / 1000;
            input.flow_time_ms = simulated_time / 1000 - 10;
            input.flow_rad_s[1] = 0;
            // Actual 20 ms optical integration uses terrain-relative clearance,
            // while the estimator's room-height datum must stay unchanged.
            for (unsigned sample = 0; sample < 5; ++sample) {
                const float stamp = time - (sample + .5f) * dt;
                const float clearance = launch_range + height(stamp, scenario).position - terrain(stamp, scenario);
                input.flow_rad_s[1] += north(stamp).velocity / clearance / 5;
            }
            if (time >= 60 && time < 61.5f) {
                if (named(scenario, "stale") || named(scenario, "gap-step")) {
                    input.fresh &= ~AP_EKF_RANGE;
                } else if (named(scenario, "invalid")) {
                    const float invalid[] = {0, -1, std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity()};
                    input.range_m = invalid[(tick / 5) % 4];
                }
            }
        }
        assert(ap_ekf_update(&input, &state));
        if (tick == 10000) {
            initial_down = state.position_ned_m[2];
            initial_north = state.position_ned_m[0];
            available_memory = ap_backend_available_memory();
        }
        if (time < 40) {
            continue;
        }
        assert(state.initialized && state.heading_aligned && !state.faults);
        assert(ap_backend_available_memory() == available_memory);
        for (unsigned axis = 0; axis < 3; ++axis) {
            assert(std::isfinite(state.position_ned_m[axis]) && std::isfinite(state.velocity_ned_mps[axis]));
        }
        if (tick == 13500) { // 54 s: airborne before the first disturbance.
            reset_count = state.position_d_reset_count;
            previous_down = state.position_ned_m[2];
            initial_terrain = state.terrain_offset_down_m;
        }
        if (time >= 54) {
            const float error = std::fabs(state.position_ned_m[2] - initial_down + z.position);
            if (error > maximum_height_error) {
                maximum_height_error = error;
                error_time = time;
            }
            maximum_velocity_error = std::max(maximum_velocity_error,
                std::fabs(state.velocity_ned_mps[2] + z.velocity));
            maximum_north_error = std::max(maximum_north_error,
                std::fabs(state.position_ned_m[0] - initial_north - x.position));
            maximum_north_velocity_error = std::max(maximum_north_velocity_error,
                std::fabs(state.velocity_ned_mps[0] - x.velocity));
            maximum_height_age = std::max(maximum_height_age, state.height_acceptance_age_ms);
            maximum_down_jump = std::max(maximum_down_jump, std::fabs(state.position_ned_m[2] - previous_down));
            previous_down = state.position_ned_m[2];
            if (named(scenario, "slope")) {
                if (!saw_expiry) {
                    maximum_pre_expiry_error = std::max(maximum_pre_expiry_error, error);
                }
                if (state.terrain_transition) {
                    if (!saw_transition) {
                        first_transition_s = time;
                    }
                    saw_transition = true;
                    if (previous_transition) {
                        assert(state.terrain_transition_age_ms >= previous_transition_age);
                        // Deliberately withheld range is not a height pass. This
                        // also catches timeout/bad-IMU paths fabricating freshness.
                        assert(state.height_acceptance_age_ms >= previous_height_age);
                    }
                    assert(bool(state.terrain_expired) == (state.terrain_transition_age_ms >= 1500));
                    if (state.terrain_expired && !saw_expiry) {
                        first_expiry_s = time;
                        saw_expiry = true;
                    }
                }
                if (saw_expiry && time >= 75) {
                    // The long unconfirmed ramp supplies no independent room
                    // datum, even once flat. A plateau cannot prove its offset.
                    saw_reacquisition |= !state.terrain_transition && state.height_acceptance_age_ms <= 300;
                    assert(state.terrain_transition && state.terrain_expired);
                    assert(state.height_acceptance_age_ms > 300);
                }
                previous_transition = state.terrain_transition;
                previous_transition_age = state.terrain_transition_age_ms;
                previous_height_age = state.height_acceptance_age_ms;
            }
            if (state.position_d_reset_count != reset_count) {
                ++unexpected_resets;
                reset_count = state.position_d_reset_count;
            }
            if ((named(scenario, "stale") || named(scenario, "invalid") || named(scenario, "gap-step")) &&
                time > 60.9f && time < 61.5f) {
                // Upstream may retain an earlier good DAL sample for 500 ms.
                // Afterwards missing/invalid packets cannot refresh acceptance.
                if (state.height_acceptance_age_ms <= 300) {
                    std::printf("Terrain %s at %.3f s: missing/invalid range still reports height age %u ms\n",
                        scenario, double(time), unsigned(state.height_acceptance_age_ms));
                    assert(state.height_acceptance_age_ms > 300);
                }
                observed_stale_height = true;
            }
            if (named(scenario, "gap-step") && time >= 61.5f && state.height_acceptance_age_ms <= 300) {
                // A fresh packet over a different surface must not fabricate
                // a valid room-height update by snapping the vehicle datum.
                const float offset_error = std::fabs(state.terrain_offset_down_m - initial_terrain + .8f);
                if (state.terrain_transition || state.terrain_expired || offset_error >= .15f) {
                    std::printf("Gap-step falsely fresh at %.3f s: age %u ms, offset error %.4f m, transition %u expired %u\n",
                        double(time), unsigned(state.height_acceptance_age_ms), double(offset_error),
                        unsigned(state.terrain_transition), unsigned(state.terrain_expired));
                    assert(false);
                }
                gap_height_reacquired = true;
            }
            if (time >= 106) {
                if (!named(scenario, "gap-step") && !named(scenario, "slope")) {
                    assert(state.healthy && state.height_acceptance_age_ms <= 300);
                }
                final_height_error = std::max(final_height_error, error);
            }
        }
    }
    std::printf("Terrain %s: max room-height error %.4f m at %.3f s; final %.4f m; "
        "max north error %.4f m / %.4f m/s; max Vd error %.4f m/s; max accepted-height age %u ms; height resets %u\n",
        scenario, double(maximum_height_error), double(error_time), double(final_height_error),
        double(maximum_north_error), double(maximum_north_velocity_error), double(maximum_velocity_error),
        unsigned(maximum_height_age), unexpected_resets);
    assert(unexpected_resets == 0);
    if (named(scenario, "slope")) {
        std::printf("Slope qualification: classified %.3f s, expired %.3f s, plateau reacquired %u, "
            "pre-expiry room-height error %.4f m, largest output-height increment %.4f m\n",
            double(first_transition_s), double(first_expiry_s), unsigned(saw_reacquisition),
            double(maximum_pre_expiry_error), double(maximum_down_jump));
        assert(saw_transition && saw_expiry && !saw_reacquisition);
        assert(first_expiry_s - first_transition_s >= 1.48f && first_expiry_s - first_transition_s <= 1.65f);
        assert(maximum_pre_expiry_error < .30f && maximum_down_jump < .10f);
    } else if (!named(scenario, "gap-step")) {
        assert(maximum_height_error < .30f && final_height_error < .20f);
    }
    // Height rejection must retain the actual terrain-relative optical scale;
    // replacing raw clearance with room altitude would accumulate lateral error.
    if (!named(scenario, "slope") && !named(scenario, "gap-step")) {
        assert(maximum_north_error < .50f && maximum_north_velocity_error < .35f);
    }
    assert((!named(scenario, "stale") && !named(scenario, "invalid") && !named(scenario, "gap-step")) || observed_stale_height);
    if (named(scenario, "gap-step")) {
        // An unobserved surface change during a sensor gap is ambiguous. It may
        // remain invalid, but must not claim a recent pass or reset room height.
        std::printf("Gap-step qualification: qualified reacquisition %u, largest output-height increment %.4f m\n",
            unsigned(gap_height_reacquired), double(maximum_down_jump));
        assert(maximum_down_jump < .10f);
    }
    return 0;
}
