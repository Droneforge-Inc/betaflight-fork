#include "ap_control.h"
#include "ap_ekf.h"
#include "runtime.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/control.h>
#include <Filter/LowPassFilter2p.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

// Host validation must exercise the G4 position representation, not the empty
// HAL board's 2 MiB default, which selects software double arithmetic on Cortex-M4.
static_assert(sizeof(postype_t) == sizeof(float));

static uint64_t simulated_time;
extern const AP_HAL::HAL &hal;
uint64_t ap_runtime_time_us() { return simulated_time; }
void ap_runtime_panic(const char *reason)
{
    std::fprintf(stderr, "AP panic: %s\n", reason);
    std::abort();
}

static void test_allocator()
{
    const uint32_t capacity = ap_backend_available_memory();
    assert(!ap_backend_calloc(std::numeric_limits<size_t>::max(), 2));
    assert(!ap_backend_malloc(std::numeric_limits<size_t>::max()));
    void *first = ap_backend_malloc(97);
    void *second = ap_backend_malloc(193);
    assert(first && second);
    std::memset(first, 0xa5, 97);
    ap_backend_free(first);
    void *reused = ap_backend_malloc(97);
    assert(reused);
    for (unsigned i = 0; i < 97; ++i) {
        assert(static_cast<unsigned char *>(reused)[i] == 0);
    }
    ap_backend_free(second);
    ap_backend_free(reused);
    assert(ap_backend_available_memory() == capacity);
    void *whole = ap_backend_malloc(capacity);
    assert(whole && !ap_backend_malloc(1));
    ap_backend_free(whole);
    assert(ap_backend_available_memory() == capacity);
}

static void test_formatting()
{
    ap_backend_runtime_init();
    char message[64];
    const char *expected = "GPS[2] vertical 1.25 (needs 0.3)";
    const int length = hal.util->snprintf(message, sizeof(message), "GPS[%u] vertical %.2f (%s)",
                                        2u, 1.25, "needs 0.3");
    assert(length == int(std::strlen(expected)) && std::strcmp(message, expected) == 0);
    assert(hal.util->snprintf(message, 5, "%s", expected) == length);
    assert(std::strcmp(message, "GPS[") == 0);
    assert(hal.util->snprintf(nullptr, 0, "%s", expected) == length);
}

int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const bool indoor = argc == 1 || std::atoi(argv[1]) != 0;
    struct Scenario {
        const char *name;
        float bias_mss;
        bool startup_bias;
        bool noisy;
    };
    const Scenario scenarios[] = {
        {"clean", 0, false, false},
        {"startup-bias", 1, true, false},
        {"motor-bias-down", 1, false, false},
        {"motor-bias-up", -1, false, false},
        {"motor-bias-0.15g", 1.4709975f, false, false},
        {"noisy", 0, false, true},
    };
    const char *name = argc > 2 ? argv[2] : "clean";
    const Scenario *scenario = nullptr;
    for (const auto &candidate : scenarios) {
        if (std::strcmp(name, candidate.name) == 0) {
            scenario = &candidate;
        }
    }
    assert(scenario && (indoor || scenario == &scenarios[0]));
    test_allocator();
    test_formatting();
    assert(ap_control_init(0.35f));
    assert(ap_ekf_init(250, indoor));
    constexpr float dt = 0.004f, gravity = 9.80665f, hover = 0.35f;
    LowPassFilter2pVector3f controller_acceleration_filter(1.0f / dt, 20.0f);
    controller_acceleration_filter.reset(Vector3f(0, 0, -gravity));
    ap_ekf_input_t sensors{};
    ap_ekf_output_t state{};
    ap_control_input input{};
    ap_control_output output{};
    sensors.dt_s = input.dt_s = dt;
    sensors.heading_error_rad = 0.08726646f;
    sensors.flow_quality = 255;
    sensors.magnetic_field_mgauss[0] = 200;
    sensors.magnetic_field_mgauss[2] = 400;
    sensors.latitude_e7 = 374000000;
    sensors.longitude_e7 = -1220000000;
    sensors.gps_fix = 3;
    sensors.gps_satellites = 16;
    sensors.gps_horizontal_accuracy_m = 0.5f;
    sensors.gps_vertical_accuracy_m = 0.8f;
    sensors.gps_speed_accuracy_mps = 0.1f;
    input.lateral_control_enabled = true;
    float altitude = 0, velocity = 0, acceleration = 0, achieved_thrust = 0;
    float peak_height = 0, hold_squared_error = 0, hover_at_degradation = 0;
    float first_degraded_s = -1;
    unsigned hold_samples = 0;
    uint32_t max_height_age_ms = 0;
    uint32_t random_state = 420;
    auto uniform = [&]() {
        random_state = 1664525u * random_state + 1013904223u;
        return float(random_state >> 8) / 16777216.0f - 0.5f;
    };
    uint32_t available_after_alignment = 0;
    const unsigned last_tick = indoor ? 19000 : 17500;
    for (unsigned tick = 1; tick <= last_tick; ++tick) {
        simulated_time = uint64_t(tick) * 4000;
        const float flight_time = (float(tick) - 10250) * dt;
        sensors.time_us = input.time_us = simulated_time;
        sensors.armed = input.armed = tick >= 10250 && (!indoor || flight_time < 34);
        sensors.automatic_throttle = true;
        sensors.takeoff_expected = tick >= 10250 && tick < 11000;
        // Corrupt the measurement, never the physical plant.
        const float bias = scenario->startup_bias || sensors.armed ? scenario->bias_mss : 0;
        const float accel_noise = scenario->noisy ? uniform() * 1.0392305f : 0; // 0.30 m/s² RMS
        sensors.delta_velocity_mps[2] = (-gravity - acceleration + bias + accel_noise) * dt;
        const Vector3f filtered_acceleration_body = controller_acceleration_filter.apply(
            Vector3f(0, 0, sensors.delta_velocity_mps[2] / dt));
        sensors.range_m = 0.065f + altitude;
        if (scenario->noisy) {
            sensors.range_m = fmaxf(0.025f, sensors.range_m + uniform() * 0.06928203f); // 2 cm RMS
        }
        sensors.baro_altitude_m = altitude;
        sensors.altitude_cm = 1000 + std::lround(altitude * 100);
        sensors.gps_velocity_ned_mps[2] = -velocity;
        sensors.fresh = tick % 5 == 0 ? AP_EKF_RANGE | AP_EKF_BARO : 0;
        if (tick % 5 == 0) {
            sensors.fresh |= indoor ? AP_EKF_FLOW | AP_EKF_HEADING : AP_EKF_GPS | AP_EKF_COMPASS;
        }
        sensors.heading_time_ms = simulated_time / 1000;
        sensors.flow_time_ms = simulated_time / 1000 - (tick >= 5 ? 10 : 0);
        assert(ap_ekf_update(&sensors, &state));
        if (tick >= 10000) {
            assert(state.initialized && state.healthy && state.heading_aligned && !state.faults);
            if (tick == 10000) {
                available_after_alignment = ap_backend_available_memory();
            }
            assert(ap_backend_available_memory() == available_after_alignment);
            if (indoor) {
                assert(std::isfinite(state.height_innovation_m) && std::isfinite(state.height_test_ratio));
                assert(state.height_acceptance_age_ms <= 300);
                max_height_age_ms = std::max(max_height_age_ms, state.height_acceptance_age_ms);
            }
        }
        std::memcpy(input.position_ned_m, state.position_ned_m, sizeof(input.position_ned_m));
        std::memcpy(input.velocity_ned_ms, state.velocity_ned_mps, sizeof(input.velocity_ned_ms));
        std::memcpy(input.quaternion_body_to_ned, state.quaternion, sizeof(input.quaternion_body_to_ned));
        if (!state.initialized) {
            input.quaternion_body_to_ned[0] = 1;
        }
        // Match AP INS -> AHRS: filter measured body acceleration separately
        // from the EKF increments, then subtract learned bias and rotate. Never
        // substitute plant truth, which would hide the motor-on bias failure.
        Matrix3f body_to_ned;
        Quaternion(input.quaternion_body_to_ned[0], input.quaternion_body_to_ned[1],
                   input.quaternion_body_to_ned[2], input.quaternion_body_to_ned[3]).rotation_matrix(body_to_ned);
        const Vector3f force = body_to_ned * (filtered_acceleration_body -
            Vector3f(state.accel_bias_body_mps2[0], state.accel_bias_body_mps2[1], state.accel_bias_body_mps2[2]));
        for (unsigned axis = 0; axis < 3; ++axis) {
            input.specific_force_ned_mss[axis] = force[axis];
        }
        for (unsigned axis = 0; axis < 3; ++axis) {
            input.gyro_body_rads[axis] = -state.gyro_bias_body_rads[axis];
        }
        input.vertical_position_rate_mps = state.vertical_position_rate_mps;
        input.vertical_degraded = state.vertical_degraded;
        input.reference_position_ned_m[2] = sensors.armed ? -1 : 0;
        if (indoor) {
            float target_height = 0, target_velocity = 0, target_acceleration = 0;
            if (flight_time >= 0 && flight_time < 3) {
                constexpr float omega = M_PI / 3;
                target_height = 0.5f * (1 - cosf(omega * flight_time));
                target_velocity = 0.5f * omega * sinf(omega * flight_time);
                target_acceleration = 0.5f * omega * omega * cosf(omega * flight_time);
            } else if (flight_time >= 3 && flight_time < 25) {
                target_height = 1;
            } else if (flight_time >= 25 && sensors.armed) {
                // Continue descending through the floor until disarm; stopping
                // the target above touchdown can leave the vehicle hovering.
                target_height = 1 - 0.25f * (flight_time - 25);
                target_velocity = -0.25f;
            }
            input.reference_position_ned_m[2] = -target_height;
            input.reference_velocity_ned_ms[2] = -target_velocity;
            input.reference_acceleration_ned_mss[2] = -target_acceleration;
        }
        input.landed = altitude <= 0;
        input.achieved_thrust_valid = true;
        input.achieved_collective_thrust = achieved_thrust;
        input.position_ne_reset_count = state.position_ne_reset_count;
        input.position_d_reset_count = state.position_d_reset_count;
        input.throttle_lower_limited = output.collective_thrust <= 0;
        input.throttle_upper_limited = output.collective_thrust >= 1;
        assert(ap_control_step(&input, &output));
        assert(output.active == bool(input.armed));
        assert(output.collective_thrust >= 0 && output.collective_thrust <= 1);
        if (indoor && sensors.armed) {
            peak_height = fmaxf(peak_height, altitude);
            if (state.vertical_degraded && first_degraded_s < 0) {
                first_degraded_s = flight_time;
                hover_at_degradation = output.hover_thrust;
            }
            if (state.vertical_degraded) {
                assert(output.vertical_degraded);
                assert(output.vertical_velocity_mps == state.vertical_position_rate_mps);
                assert(output.hover_thrust == hover_at_degradation);
            }
            if (flight_time >= 10 && flight_time < 25) {
                const float error = altitude - 1;
                hold_squared_error += error * error;
                ++hold_samples;
            }
            if (flight_time >= 33) {
                assert(altitude < 0.05f);
            }
        }
        if (indoor && flight_time >= 34) {
            assert(!output.active && output.collective_thrust == 0);
            if (state.vertical_degraded) {
                assert(output.vertical_degraded);
                assert(output.vertical_velocity_mps == state.vertical_position_rate_mps);
            }
        }
        achieved_thrust += dt / (dt + 0.035f) * (output.collective_thrust - achieved_thrust);
        acceleration = gravity * (achieved_thrust / hover - 1);
        velocity += acceleration * dt;
        altitude += velocity * dt;
        if (altitude <= 0) {
            altitude = velocity = acceleration = 0;
        }
    }
    std::printf("EKF3 + AP controller (%s/%s): peak %.4f m, hold RMSE %.4f m, degraded %.3f s, "
                "height age %u ms, final height %.4f m, arena free %u bytes\n",
                indoor ? "flow" : "GPS", name, double(peak_height),
                hold_samples ? double(sqrtf(hold_squared_error / hold_samples)) : 0.0,
                double(first_degraded_s), unsigned(max_height_age_ms), double(altitude),
                unsigned(available_after_alignment));
    if (indoor) {
        const bool motor_bias = scenario->bias_mss != 0 && !scenario->startup_bias;
        // A motor-on bias alone is not proof of vibration. The upstream
        // evidence policy decides activation; never require the removed
        // 0.2-second velocity-disagreement shortcut to rescue this fixture.
        if (!motor_bias) {
            assert(first_degraded_s < 0);
        }
        assert(peak_height < (motor_bias ? 1.5f : 1.15f));
        assert(hold_samples && sqrtf(hold_squared_error / hold_samples) < (motor_bias ? 0.15f : 0.05f));
        assert(altitude == 0 && velocity == 0);
    } else {
        assert(fabsf(altitude - 1) < 0.05f);
        assert(fabsf(state.position_ned_m[2] + altitude) < 0.05f);
        assert(fabsf(velocity) < 0.05f);
    }
    ap_control_destroy();
    assert(ap_control_init(hover));
    assert(ap_backend_available_memory() == available_after_alignment);
}
