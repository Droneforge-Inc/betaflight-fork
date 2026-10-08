#pragma once

#include <Filter/LowPassFilter.h>
#include <stdint.h>

/* ArduCopter's check_vibration() evidence. "Variance" is getVariances()'s
 * normalized velocity innovation ratio, not the state covariance or the gap
 * between ordinary and complementary velocity estimates.
 */
struct ap_vibration_input {
    float velocity_variance;
    float velocity_innovation_down;
    float position_innovation_down;
    bool variances_valid;
    bool innovations_valid;
    bool vibration_affected;
    bool armed;
    bool automatic_throttle;
};

class APVibrationPolicy {
public:
    APVibrationPolicy() { reset(); }
    void reset();
    bool due(uint64_t now_us) const { return now_us - last_check_us >= 100000; }
    void update(uint64_t now_us, const ap_vibration_input &input);
    bool active() const { return high_vibes; }

private:
    LowPassFilterFloat velocity_variance{5.0f}; // ArduCopter FS_EKF_FILT_DEFAULT
    uint64_t last_check_us;
    uint64_t last_variance_us;
    uint32_t start_ms;
    uint32_t clear_ms;
    bool high_vibes;
};
