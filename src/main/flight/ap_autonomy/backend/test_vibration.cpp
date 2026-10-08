#include "ap_vibration.h"
#include "runtime.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>

uint64_t ap_runtime_time_us() { return 0; }
void ap_runtime_panic(const char *reason)
{
    std::fprintf(stderr, "AP panic: %s\n", reason);
    std::abort();
}

static ap_vibration_input bad_innovations()
{
    return {2.0f, 0.5f, 0.5f, true, true, false, true, true};
}

static void tick(APVibrationPolicy &policy, uint64_t &time_us, const ap_vibration_input &input)
{
    time_us += 100000;
    assert(policy.due(time_us));
    policy.update(time_us, input);
    assert(!policy.due(time_us + 99999));
}

static void test_evidence()
{
    // A positive height innovation alone, or a velocity-estimate disagreement
    // of any magnitude, is insufficient: no disagreement enters this policy.
    for (unsigned rejection = 0; rejection < 7; ++rejection) {
        APVibrationPolicy policy;
        uint64_t time_us = 0;
        auto input = bad_innovations();
        switch (rejection) {
        case 0: input.velocity_innovation_down = 0; break;
        case 1: input.position_innovation_down = -0.5f; break;
        case 2: input.variances_valid = false; break;
        case 3: input.innovations_valid = false; break;
        case 4: input.velocity_variance = 1; break;
        case 5: input.armed = false; break;
        case 6: input.automatic_throttle = false; break;
        }
        for (unsigned i = 0; i < 30; ++i) {
            tick(policy, time_us, input);
            assert(!policy.active());
        }
    }

    APVibrationPolicy policy;
    uint64_t time_us = 0;
    auto input = bad_innovations();
    input.variances_valid = input.innovations_valid = false;
    input.vibration_affected = true;
    for (unsigned i = 0; i < 10; ++i) {
        tick(policy, time_us, input);
        assert(!policy.active());
    }
    tick(policy, time_us, input);
    assert(policy.active()); // EKF badIMU evidence is independently sufficient.
}

static void test_filter_and_persistence()
{
    APVibrationPolicy policy;
    uint64_t time_us = 0;
    auto input = bad_innovations();
    input.velocity_variance = 0.2f;
    tick(policy, time_us, input);
    input.velocity_variance = 1.2f;
    tick(policy, time_us, input); // 5 Hz filter remains below one on this tick.
    for (unsigned i = 0; i < 10; ++i) {
        tick(policy, time_us, input);
        assert(!policy.active());
    }
    tick(policy, time_us, input);
    assert(policy.active());
    tick(policy, time_us, input); // Refresh upstream's recovery timer while bad.
    input.velocity_innovation_down = 0;
    for (unsigned i = 0; i < 150; ++i) {
        tick(policy, time_us, input);
        assert(policy.active());
    }
    tick(policy, time_us, input);
    assert(!policy.active()); // Recovery is allowed in flight after >15 s clear.

    // Invalid observations break entry persistence rather than reusing the last
    // valid high ratio. Recovery and rearming do not create a permanent latch.
    policy.reset();
    time_us = 0;
    input = bad_innovations();
    for (unsigned i = 0; i < 9; ++i) {
        tick(policy, time_us, input);
    }
    input.innovations_valid = false;
    tick(policy, time_us, input);
    input.innovations_valid = true;
    for (unsigned i = 0; i < 10; ++i) {
        tick(policy, time_us, input);
        assert(!policy.active());
    }
    tick(policy, time_us, input);
    assert(policy.active());
    policy.reset();
    assert(!policy.active());
}

// Executable reference transcribed from pinned ArduCopter/ekf_check.cpp:
// ekf_over_threshold()'s variance filter, followed by check_vibration().
// Deliberately keep its branching and timer initialization, including clear_ms
// set to zero on activation, so later policy edits cannot silently diverge.
struct UpstreamReference {
    LowPassFilterFloat vel_variance_filt{5.0f};
    uint64_t last_ekf_check_us = 0;
    uint32_t start_ms = 0, clear_ms = 0;
    bool high_vibes = false;

    UpstreamReference() { vel_variance_filt.reset(); }
    bool update(uint64_t now_us, const ap_vibration_input &input)
    {
        if (input.variances_valid) {
            const float dt = (now_us - last_ekf_check_us) * 1e-6f;
            vel_variance_filt.apply(input.velocity_variance, dt);
            last_ekf_check_us = now_us;
        }
        bool innovation_checks_valid = true;
        if (!input.innovations_valid) {
            innovation_checks_valid = false;
        }
        const bool innov_velD_posD_positive = is_positive(input.velocity_innovation_down) &&
                                             is_positive(input.position_innovation_down);
        if (!input.variances_valid) {
            innovation_checks_valid = false;
        }
        const bool bad_vibe_detected = (innovation_checks_valid && innov_velD_posD_positive &&
                                       vel_variance_filt.get() > 1.0f) || input.vibration_affected;
        const bool do_bad_vibe_actions = bad_vibe_detected && input.armed && input.automatic_throttle;
        const uint32_t now = now_us / 1000;
        if (!high_vibes) {
            if (!do_bad_vibe_actions) {
                start_ms = now;
            }
            if (now - start_ms > 1000) {
                clear_ms = 0;
                high_vibes = true;
            }
        } else {
            if (do_bad_vibe_actions) {
                clear_ms = now;
            }
            if (now - clear_ms > 15000) {
                start_ms = 0;
                high_vibes = false;
                clear_ms = 0;
            }
        }
        return high_vibes;
    }
};

static void test_upstream_parity(uint64_t start_us)
{
    APVibrationPolicy policy;
    UpstreamReference upstream;
    uint64_t time_us = start_us;
    for (unsigned i = 0; i < 3000; ++i) {
        auto input = bad_innovations();
        input.armed = i % 1300 > 20;
        input.automatic_throttle = i % 1100 > 20;
        input.velocity_variance = i % 600 < 40 ? 2 : 0.2f;
        input.velocity_innovation_down = i % 900 > 20 ? 0.5f : -0.5f;
        input.position_innovation_down = i % 950 > 20 ? 0.5f : 0;
        input.variances_valid = i % 870 > 20;
        input.innovations_valid = i % 830 > 20;
        input.vibration_affected = i % 400 < 30;
        time_us += i % 10 == 0 ? 120000 : 100000;
        policy.update(time_us, input);
        assert(policy.active() == upstream.update(time_us, input));
    }
}

static void test_immediate_clear_after_activation()
{
    APVibrationPolicy policy;
    UpstreamReference upstream;
    uint64_t time_us = 60000000;
    auto input = bad_innovations();
    input.armed = false;
    policy.update(time_us, input);
    upstream.update(time_us, input);
    input.armed = true;
    for (unsigned i = 0; i < 11; ++i) {
        tick(policy, time_us, input);
        assert(policy.active() == upstream.update(time_us, input));
    }
    assert(policy.active());
    input.velocity_innovation_down = 0;
    tick(policy, time_us, input);
    assert(policy.active());
    // Documented correction to pinned OG: its zero clear_ms skips recovery
    // when the first post-activation check is clear and uptime exceeds 15 s.
    assert(!upstream.update(time_us, input));
    for (unsigned i = 1; i < 150; ++i) {
        tick(policy, time_us, input);
        assert(policy.active());
    }
    tick(policy, time_us, input);
    assert(!policy.active());
}

int main()
{
    test_evidence();
    test_filter_and_persistence();
    test_upstream_parity(0);
    test_upstream_parity((uint64_t(UINT32_MAX) - 5000) * 1000); // millis rollover
    test_immediate_clear_after_activation();
    std::puts("AP vibration policy: evidence, 10 Hz schedule, filtering, persistence, recovery and upstream parity passed");
}
