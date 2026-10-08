#include "ap_ekf.h"

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

static void unavailable()
{
    ap_ekf_vertical_diagnostics_t value;
    std::memset(&value, 0xa5, sizeof(value));
    const auto before = value;
    assert(!ap_ekf_get_vertical_diagnostics(&value));
    assert(std::memcmp(&before, &value, sizeof(value)) == 0);
}

static void same_event(const ap_ekf_vertical_diagnostics_t &a, const ap_ekf_vertical_diagnostics_t &b)
{
    constexpr uint32_t held_flags = AP_EKF_DIAG_HEIGHT_EVENT | AP_EKF_DIAG_HEIGHT_SAMPLE |
        AP_EKF_DIAG_HEIGHT_RANGE | AP_EKF_DIAG_EVENT_BAD_IMU | AP_EKF_DIAG_EVENT_BIAS_INHIBITED |
        AP_EKF_DIAG_EVENT_BIAS_Z_INHIBITED | AP_EKF_DIAG_EVENT_TERRAIN;
    assert(a.fusion_time_ms == b.fusion_time_ms);
    assert(a.height_sample_time_ms == b.height_sample_time_ms);
    assert(a.height_fusion_sequence == b.height_fusion_sequence);
    assert((a.flags & held_flags) == (b.flags & held_flags));
    assert(a.height_observation_down_m == b.height_observation_down_m);
    assert(a.height_variance_m2 == b.height_variance_m2);
}

int main(int argc, char **argv)
{
    assert(argc == 4); // off/on/toggle, clean/motor-bias, binary normal-output path
    const bool toggle = std::strcmp(argv[1], "toggle") == 0;
    const bool requested = toggle || std::strcmp(argv[1], "on") == 0;
    assert(requested || std::strcmp(argv[1], "off") == 0);
    const bool biased = std::strcmp(argv[2], "motor-bias") == 0;
    assert(biased || std::strcmp(argv[2], "clean") == 0);
    FILE *outputs = std::fopen(argv[3], "wb");
    assert(outputs);
    unavailable();
    ap_ekf_set_vertical_diagnostics(requested);
    unavailable();
    assert(ap_ekf_init(250, 1));
    unavailable();
    assert(!ap_ekf_get_vertical_diagnostics(nullptr));

    ap_ekf_input_t input{};
    input.dt_s = 0.004f;
    input.automatic_throttle = true;
    input.heading_error_rad = 0.08726646f;
    input.flow_quality = 255;
    ap_ekf_vertical_diagnostics_t previous{};
    unsigned snapshots = 0, successes = 0, held = 0;
    unsigned nonzero_observer = 0, nonzero_complementary = 0, healthy = 0;
    bool enabled = requested;
    for (unsigned tick = 1; tick <= 17000; ++tick) {
        simulated_time = uint64_t(tick) * 4000;
        const float time = tick * input.dt_s;
        // A prescribed, physically consistent one-metre rise over four seconds,
        // then a fixed-height hold. The IMU bias changes only the measurement.
        const float u = std::clamp((time - 42.0f) / 4.0f, 0.0f, 1.0f);
        const float height = u*u*u * (10.0f - 15.0f*u + 6.0f*u*u);
        const float acceleration = (60.0f*u - 180.0f*u*u + 120.0f*u*u*u) / 16.0f;
        input.time_us = simulated_time;
        input.armed = time >= 41;
        input.takeoff_expected = time >= 41 && time < 46;
        input.delta_velocity_mps[2] = (-9.80665f - acceleration +
                                      (biased && input.armed ? 1.0f : 0.0f)) * input.dt_s;
        input.range_m = 0.065f + height;
        input.fresh = tick % 5 == 0 ? AP_EKF_RANGE | AP_EKF_FLOW | AP_EKF_HEADING : 0;
        input.flow_time_ms = input.heading_time_ms = simulated_time / 1000;
        if (toggle && (tick == 12250 || tick == 12300)) {
            enabled = tick == 12300;
            ap_ekf_set_vertical_diagnostics(enabled);
            unavailable();
            previous = {};
        }
        // Repeating the same enable state must not invalidate a valid snapshot.
        ap_ekf_set_vertical_diagnostics(enabled);
        ap_ekf_output_t state;
        std::memset(&state, 0, sizeof(state));
        assert(ap_ekf_update(&input, &state));
        assert(std::fwrite(&state, sizeof(state), 1, outputs) == 1);
        healthy += input.armed && state.healthy;
        ap_ekf_vertical_diagnostics_t diagnostic{};
        const bool available = ap_ekf_get_vertical_diagnostics(&diagnostic);
#if EK3_FEATURE_DF_DIAGNOSTICS
        if (enabled && time > 40) {
            assert(available);
        }
#else
        assert(!available);
#endif
        if (!available) {
            unavailable();
            continue;
        }
        assert(enabled);
        ++snapshots;
        assert(diagnostic.flags & AP_EKF_DIAG_ENABLED);
        assert(diagnostic.flags & AP_EKF_DIAG_CORE_UPDATED);
        ap_ekf_vertical_diagnostics_t repeated{};
        assert(ap_ekf_get_vertical_diagnostics(&repeated));
        assert(std::memcmp(&diagnostic, &repeated, sizeof(diagnostic)) == 0);
        if (diagnostic.flags & AP_EKF_DIAG_HEIGHT_EVENT) {
            assert(diagnostic.fusion_time_ms <= simulated_time / 1000);
            assert(diagnostic.flags & AP_EKF_DIAG_HEIGHT_SAMPLE);
            assert(diagnostic.flags & AP_EKF_DIAG_HEIGHT_RANGE);
            assert(diagnostic.height_sample_time_ms <= simulated_time / 1000);
            assert(diagnostic.height_variance_m2 > 0);
        }
        if (diagnostic.flags & AP_EKF_DIAG_HEIGHT_FUSED) {
            ++successes;
            assert(diagnostic.flags & AP_EKF_DIAG_HEIGHT_ATTEMPT);
            assert(diagnostic.flags & AP_EKF_DIAG_HEIGHT_EVENT);
            assert(diagnostic.height_fusion_sequence == previous.height_fusion_sequence + 1);
        } else {
            ++held;
            same_event(diagnostic, previous);
        }
        if (!(diagnostic.flags & AP_EKF_DIAG_PREDICTED)) {
            assert(diagnostic.output_position_delta_down_m == 0);
            assert(diagnostic.output_velocity_delta_down_mps == 0);
        }
        assert(std::isfinite(diagnostic.output_delta_velocity_down_mps));
        assert(std::isfinite(diagnostic.complementary_accel_correction_mps2));
        nonzero_observer += std::fabs(diagnostic.output_velocity_delta_down_mps) > 1e-6f;
        nonzero_complementary += std::fabs(diagnostic.complementary_accel_correction_mps2) > 1e-6f;
        previous = diagnostic;
    }
    assert(std::fclose(outputs) == 0);
    assert(healthy > 6000);
#if EK3_FEATURE_DF_DIAGNOSTICS
    if (requested) {
        assert(snapshots > 10000 && successes > 100 && held > 100);
        assert(nonzero_observer > 100 && nonzero_complementary > 100);
    }
#endif
    assert(!ap_ekf_update(nullptr, nullptr));
    unavailable();
    ap_ekf_set_vertical_diagnostics(0);
    unavailable();
    std::printf("%s %s: snapshots=%u height_fusions=%u held=%u "
                "observer_corrections=%u complementary=%u healthy=%u\n",
                argv[1], argv[2], snapshots, successes, held,
                nonzero_observer, nonzero_complementary, healthy);
}
