#pragma once

#include <stdint.h>

// A conditional terrain policy, not an independent altitude sensor. Range and
// IMU cannot distinguish an arbitrary terrain slope from vertical inertial drift.
// The caller owns the terrain offset and must not fuse height when fuse_height
// is false, including through a timeout or bad-IMU override.
struct ap_terrain_input_t {
    uint32_t time_ms; // Unique delayed range timestamp, in the IMU clock.
    float range_down_m; // Negative, tilt/lever-arm corrected range; not room height.
    float horizontal_speed_mps;
    float velocity_variance_m2ps2;
    bool armed;
    // Independent IMU displacement interpolated at time_ms, in one continuous
    // coordinate for successive observations. Never use height-corrected pd.
    float motion_position_down_m{};
};

struct ap_terrain_result_t {
    float terrain_delta_down_m;
    uint32_t transition_age_ms;
    bool fresh;
    bool fuse_height;
    bool transition;
    bool expired;
    bool reanchor_motion;
    uint32_t reanchor_time_ms;
    float reanchor_velocity_down_mps;
};

struct ap_terrain_t {
    uint32_t previous_ms{}, transition_ms{}, window_ms{}, mean_ms{}, rate_ms{};
    float previous_range{}, previous_velocity_sigma{};
    float previous_motion{}, compensated_range{}, anchor_range{}, anchor_raw_range{}, anchor_motion_sigma{};
    float candidate_residual{}, gap_gate{};
    float sum_raw{}, sum_compensated{}, sum_time{}, mean_raw{}, mean_compensated{};
    float raw_rate{}, compensated_rate{};
    unsigned window_count{};
    uint8_t mode{};
    bool initialized{};
    ap_terrain_result_t result{};
};

// Reset only with the estimator's coordinate epoch or while disarmed. A reset
// does not change the caller-owned terrain offset.
void ap_terrain_reset(ap_terrain_t &policy);
const ap_terrain_result_t &ap_terrain_observe(ap_terrain_t &policy, const ap_terrain_input_t &sample);
