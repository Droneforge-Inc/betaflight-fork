#include "ap_vibration.h"

void APVibrationPolicy::reset()
{
    velocity_variance.reset();
    last_check_us = last_variance_us = 0;
    start_ms = clear_ms = 0;
    high_vibes = false;
}

void APVibrationPolicy::update(uint64_t now_us, const ap_vibration_input &input)
{
    last_check_us = now_us;
    if (input.variances_valid) {
        const float dt = (now_us - last_variance_us) * 1.0e-6f;
        velocity_variance.apply(input.velocity_variance, dt);
        last_variance_us = now_us;
    }

    // Keep the evidence and timer transitions aligned with ArduCopter's
    // ekf_check.cpp: ekf_over_threshold() followed by check_vibration(), 10 Hz.
    const bool positive_innovations = is_positive(input.velocity_innovation_down) &&
                                      is_positive(input.position_innovation_down);
    const bool bad_vibration = (input.variances_valid && input.innovations_valid &&
        positive_innovations && velocity_variance.get() > 1.0f) || input.vibration_affected;
    const bool do_bad_vibe_actions = bad_vibration && input.armed && input.automatic_throttle;
    const uint32_t now_ms = now_us / 1000;
    if (!high_vibes) {
        if (!do_bad_vibe_actions) {
            start_ms = now_ms;
        }
        if (now_ms - start_ms > 1000) {
            // Upstream initializes this to zero. Use the activation timestamp
            // so a disturbance clearing on the next tick still gets the full
            // 15-second recovery window on an FC running longer than 15 s.
            clear_ms = now_ms;
            high_vibes = true;
        }
    } else {
        if (do_bad_vibe_actions) {
            clear_ms = now_ms;
        }
        if (now_ms - clear_ms > 15000) {
            start_ms = 0;
            high_vibes = false;
            clear_ms = 0;
        }
    }
}
