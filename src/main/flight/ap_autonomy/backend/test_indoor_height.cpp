#include "ap_ekf.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

// Prescribed motion over a flat floor, not a controller/aircraft simulation or
// a replay of sparse flight logs. The assertions qualify aiding and validity,
// not a centimetre-level position-accuracy target.
static uint64_t simulated_time;
uint64_t ap_runtime_time_us() { return simulated_time; }
void ap_runtime_panic(const char *reason)
{
    std::fprintf(stderr, "AP panic: %s\n", reason);
    std::abort();
}

struct Motion {
    float position, velocity;
};

static Motion move(float time, float start, float duration, float distance)
{
    const float u = std::clamp((time - start) / duration, 0.0f, 1.0f);
    return {distance * u*u*u * (10 + u * (-15 + 6*u)),
            distance / duration * 30 * u*u * (1-u) * (1-u)};
}

static Motion height(float time)
{
    return move(time, 42, 3, 1.5f);
}

static Motion north(float time)
{
    return move(time, 42, 8, 1.2f);
}

int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    assert(argc == 2);
    const bool clean = std::strcmp(argv[1], "clean") == 0;
    const bool stale = std::strcmp(argv[1], "stale") == 0;
    const bool invalid = std::strcmp(argv[1], "invalid") == 0;
    assert(clean || stale || invalid || std::strcmp(argv[1], "disturbed") == 0);
    assert(ap_ekf_init(250, 1));
    constexpr float dt = .004f, gravity = 9.80665f, floor_range = .065f;
    ap_ekf_input_t input{};
    input.dt_s = dt;
    input.automatic_throttle = true;
    input.sensor_position_body_m[2] = .035f;
    input.flow_quality = 200;
    input.heading_error_rad = .05f;
    ap_ekf_output_t state{};
    uint32_t maximum_aided_age = 0, maximum_outage_age = 0;
    unsigned aided_updates = 0, stale_updates = 0, recovered_updates = 0;
    float initial_down = 0, maximum_height_error = 0;
    for (unsigned tick = 1; tick <= 17000; ++tick) {
        const float time = tick * dt;
        const Motion z = height(time), previous_z = height(time-dt);
        const Motion x = north(time), previous_x = north(time-dt);
        simulated_time = uint64_t(tick) * 4000;
        input.time_us = simulated_time;
        input.armed = time >= 41;
        input.takeoff_expected = time >= 41 && time < 46;
        input.delta_velocity_mps[0] = x.velocity - previous_x.velocity;
        input.delta_velocity_mps[2] = -(z.velocity - previous_z.velocity) - gravity * dt;
        if (!clean && input.armed) {
            // Measurement corruption only: a 0.1 g motor-on offset plus
            // deterministic vibration. Truth and range remain independent.
            input.delta_velocity_mps[2] += (1 + .3f * std::sin(2 * 3.14159265f * 73 * time)) * dt;
        }
        input.fresh = 0;
        if (tick % 5 == 0) {
            const float range_time = time - (clean ? 0 : .08f);
            input.range_m = floor_range + height(range_time).position;
            if (!clean && input.armed) {
                input.range_m += .008f * std::sin(2 * 3.14159265f * 7 * range_time);
                input.range_m = std::round(input.range_m * 100) / 100;
            }
            input.fresh = AP_EKF_RANGE | AP_EKF_FLOW | AP_EKF_HEADING;
            input.heading_time_ms = simulated_time / 1000;
            input.flow_time_ms = simulated_time / 1000 - 10;
            input.flow_rad_s[1] = 0;
            for (unsigned sample = 0; sample < 5; ++sample) {
                const float stamp = time - (sample + .5f) * dt;
                input.flow_rad_s[1] += north(stamp).velocity / (floor_range + height(stamp).position) / 5;
            }
            if (time >= 60 && time < 62.2f) {
                if (stale) {
                    input.fresh &= ~AP_EKF_RANGE;
                } else if (invalid) {
                    const float bad_range[] = {0, -1, std::numeric_limits<float>::quiet_NaN(),
                                               std::numeric_limits<float>::infinity()};
                    input.range_m = bad_range[(tick / 5) % 4];
                }
            }
        }
        assert(ap_ekf_update(&input, &state));
        if (time < 40) {
            continue;
        }
        if (tick == 10000) {
            initial_down = state.position_ned_m[2];
        }
        if (state.terrain_transition || state.terrain_expired ||
            state.terrain_transition_age_ms || state.terrain_offset_down_m != 0) {
            std::printf("Indoor %s: unexpected terrain output at %.3f s, transition=%u expired=%u age=%u offset=%.3f\n",
                        argv[1], double(time), unsigned(state.terrain_transition), unsigned(state.terrain_expired),
                        unsigned(state.terrain_transition_age_ms), double(state.terrain_offset_down_m));
            assert(false);
        }
        assert(state.initialized && state.heading_aligned && !state.faults);
        assert(std::isfinite(state.position_ned_m[2]) && std::isfinite(state.vertical_position_rate_mps));
        const bool sensor_outage = (stale || invalid) && time >= 60 && time < 62.2f;
        if (!sensor_outage && (time < 60 || time >= 64)) {
            // Continued actual height acceptance beyond the former 1.5 s
            // terrain-coast expiry is the relevant regression here.
            if (state.height_acceptance_age_ms > 300 || !state.healthy) {
                std::printf("Indoor %s: aiding unavailable at %.3f s, accepted height age=%u ms healthy=%u\n",
                            argv[1], double(time), unsigned(state.height_acceptance_age_ms), unsigned(state.healthy));
            }
            assert(state.height_acceptance_age_ms <= 300 && state.healthy);
            maximum_aided_age = std::max(maximum_aided_age, state.height_acceptance_age_ms);
            aided_updates += time >= 42 && time < 60;
            recovered_updates += time >= 64;
        }
        if (sensor_outage && time >= 60.9f) {
            // DAL can retain a good range briefly; after that, neither missing
            // nor nonfinite/negative packets may fabricate accepted-height age.
            assert(state.height_acceptance_age_ms > 300);
            maximum_outage_age = std::max(maximum_outage_age, state.height_acceptance_age_ms);
            ++stale_updates;
        }
        if (time >= 42 && time < 60) {
            maximum_height_error = std::max(maximum_height_error,
                std::fabs(state.position_ned_m[2] - initial_down + z.position));
        }
    }
    assert(aided_updates > 4000 && recovered_updates > 900);
    assert((stale || invalid) ? stale_updates > 300 && maximum_outage_age > 1000 : stale_updates == 0);
    // Invalid integrated IMU input remains a rejected backend call.
    input.time_us += 4000;
    input.delta_velocity_mps[0] = std::numeric_limits<float>::quiet_NaN();
    assert(!ap_ekf_update(&input, &state));
    std::printf("Indoor %s: aided updates=%u max accepted-height age=%u ms; outage updates=%u max age=%u ms; "
                "recovered updates=%u; measured max height error=%.3f m (no accuracy gate)\n",
                argv[1], aided_updates, maximum_aided_age, stale_updates, maximum_outage_age,
                recovered_updates, double(maximum_height_error));
}
