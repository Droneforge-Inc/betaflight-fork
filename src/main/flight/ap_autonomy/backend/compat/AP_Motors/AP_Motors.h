#pragma once
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>

/* The actual rate controller and actuator mixing remain in Betaflight. */
class AP_Motors {
public:
    struct {
        bool roll = false, pitch = false, yaw = false;
        bool throttle_lower = false, throttle_upper = false;
    } limit;
    float hover = 0.35f;
    float throttle = 0;
    LowPassFilterFloat throttle_filter;
    float throttle_avg_max = 0;
    float get_throttle_hover() const { return hover; }
    float get_throttle() const { return constrain_float(throttle_filter.get(), 0, 1); }
    float get_throttle_out() const { return get_throttle(); }
    float get_throttle_slew_rate() const { return 0; }
    void set_throttle(float value) { throttle = value; }
    void set_throttle_filter_cutoff(float hz) { throttle_filter.set_cutoff_frequency(hz); }
    void set_throttle_avg_max(float value) { throttle_avg_max = value; }
    // A call here would accidentally run AP's inner rate loop as well as BF's.
    void set_roll(float) { AP_HAL::panic("AP rate output called: Betaflight owns rate PID"); }
    void set_pitch(float) { AP_HAL::panic("AP rate output called: Betaflight owns rate PID"); }
    void set_yaw(float) { AP_HAL::panic("AP rate output called: Betaflight owns rate PID"); }
    void set_roll_ff(float) { AP_HAL::panic("AP rate output called: Betaflight owns rate PID"); }
    void set_pitch_ff(float) { AP_HAL::panic("AP rate output called: Betaflight owns rate PID"); }
    void set_yaw_ff(float) { AP_HAL::panic("AP rate output called: Betaflight owns rate PID"); }
    void update_throttle_filter(float dt, bool armed) {
        // AP_MotorsMulticopter::update_throttle_filter; Betaflight owns spool state.
        if (armed) {
            throttle_filter.apply(throttle, dt);
            if (throttle_filter.get() < 0 || throttle_filter.get() > 1) {
                throttle_filter.reset(constrain_float(throttle_filter.get(), 0, 1));
            }
        } else {
            throttle_filter.reset(0);
        }
    }
};
