#include "ap_terrain.h"

#include <algorithm>
#include <cmath>

namespace {
enum : uint8_t { STABLE, CANDIDATE, ABSORBING, GAP };
constexpr uint32_t MAX_COAST_MS = 1500;
constexpr float RANGE_SIGMA_M = 0.015f;
constexpr float MIN_TRANSLATION_MPS = 0.05f;

// Consecutive 250 ms means distinguish a continuing ramp from a plateau
// without a sliding regression or repeatedly integrating noisy range rates.
bool observe_rates(ap_terrain_t &p, uint32_t time_ms, float raw)
{
    if (!p.window_count) {
        p.window_ms = time_ms;
    }
    p.sum_raw += raw;
    p.sum_compensated += p.compensated_range;
    p.sum_time += float(time_ms - p.window_ms);
    ++p.window_count;
    if (time_ms - p.window_ms < 250) {
        return false;
    }
    const float mean_raw = p.sum_raw / p.window_count;
    const float mean_compensated = p.sum_compensated / p.window_count;
    const uint32_t mean_ms = p.window_ms + uint32_t(p.sum_time / p.window_count);
    const bool ready = p.mean_ms != 0;
    if (ready) {
        const float dt = float(mean_ms - p.mean_ms) * .001f;
        p.rate_ms = p.mean_ms + (mean_ms - p.mean_ms) / 2;
        p.raw_rate = (mean_raw - p.mean_raw) / dt;
        p.compensated_rate = (mean_compensated - p.mean_compensated) / dt;
    }
    p.mean_ms = mean_ms;
    p.mean_raw = mean_raw;
    p.mean_compensated = mean_compensated;
    p.sum_raw = p.sum_compensated = p.sum_time = 0;
    p.window_count = 0;
    return ready;
}

// Sharing these repeated stores saves 30 flash bytes with the G4 release flags.
__attribute__((noinline)) void anchor(ap_terrain_t &p)
{
    p.anchor_range = p.compensated_range;
    p.anchor_raw_range = p.previous_range;
    p.anchor_motion_sigma = 0;
}

bool valid(const ap_terrain_input_t &s)
{
    return std::isfinite(s.range_down_m) && s.range_down_m < 0 &&
        std::isfinite(s.horizontal_speed_mps) && s.horizontal_speed_mps >= 0 &&
        std::isfinite(s.velocity_variance_m2ps2) && s.velocity_variance_m2ps2 >= 0 &&
        std::isfinite(s.motion_position_down_m);
}
} // namespace

void ap_terrain_reset(ap_terrain_t &p)
{
    p = {};
}

const ap_terrain_result_t &ap_terrain_observe(ap_terrain_t &p, const ap_terrain_input_t &s)
{
    auto &result = p.result;
    result.fresh = false;
    result.terrain_delta_down_m = 0;
    if (!valid(s)) {
        result.fuse_height = false;
        return result;
    }
    if (p.initialized && int32_t(s.time_ms - p.previous_ms) <= 0) {
        return result;
    }
    result.fresh = true;
    const float velocity_sigma = std::sqrt(s.velocity_variance_m2ps2);
    const bool gap = p.initialized && s.time_ms - p.previous_ms > 250;
    if (!p.initialized || !s.armed) {
        ap_terrain_reset(p);
        p.initialized = true;
        p.previous_ms = s.time_ms;
        p.previous_range = p.compensated_range = s.range_down_m;
        p.previous_velocity_sigma = velocity_sigma;
        p.previous_motion = s.motion_position_down_m;
        anchor(p);
        observe_rates(p, s.time_ms, s.range_down_m);
        p.result.fresh = p.result.fuse_height = true;
        return p.result;
    }

    const uint32_t elapsed_ms = s.time_ms - p.previous_ms;
    const float dt = float(elapsed_ms) * .001f;
    const float delta_motion = s.motion_position_down_m - p.previous_motion;
    const float raw_delta = s.range_down_m - p.previous_range;
    p.previous_motion = s.motion_position_down_m;
    const float delta = raw_delta - delta_motion;
    const float motion_sigma = dt * std::max(p.previous_velocity_sigma, velocity_sigma);
    if (gap && p.mode != GAP) {
        // Unknown motion during missing data cannot establish a new surface.
        // Require a fresh quiet window compatible with the pre-gap geometry;
        // do not let bad-IMU/timeout fusion overrides bypass this qualification.
        if (p.mode == STABLE) {
            p.transition_ms = p.previous_ms;
            anchor(p);
        }
        p.mode = GAP;
        p.gap_gate = std::max(.1f, std::min(.3f, 2 * motion_sigma + .04f));
        p.window_count = 0;
        p.mean_ms = 0;
        p.sum_raw = p.sum_compensated = p.sum_time = 0;
    }
    p.previous_ms = s.time_ms;
    p.previous_range = s.range_down_m;
    p.previous_velocity_sigma = velocity_sigma;
    p.compensated_range += delta;
    p.anchor_motion_sigma += motion_sigma;
    const bool rates_ready = observe_rates(p, s.time_ms, s.range_down_m);
    const bool quiet = rates_ready && std::fabs(p.compensated_rate) < .025f;
    const bool raw_quiet = rates_ready && std::fabs(p.raw_rate) < .025f;
    const float residual = p.compensated_range - p.anchor_range;
    const float raw_change = s.range_down_m - p.anchor_raw_range;
    const bool translating = s.horizontal_speed_mps >= MIN_TRANSLATION_MPS;
    // Include uncertainty in the motion removed from the range difference.
    // Repeated velocity errors may be fully correlated, so add sigma * dt
    // before squaring rather than treating every sample as independent.
    const float gate = std::max(.06f, 2 * std::sqrt(2 * RANGE_SIGMA_M * RANGE_SIGMA_M +
                                                  p.anchor_motion_sigma * p.anchor_motion_sigma));
    result = {};
    result.fresh = true;

    if (p.mode == STABLE) {
        // Uncertain vertical motion cannot prove a terrain offset, but neither
        // can it justify treating coherent changed range as vehicle altitude.
        // Enter bounded ambiguity first; confirmation below still needs the
        // uncertainty gate before any terrain offset is changed.
        if ((translating && std::fabs(raw_change) > .04f && std::fabs(residual) > .06f &&
             raw_change * residual > 0 && p.raw_rate * residual > 0) ||
            (std::fabs(raw_delta) > .283f && std::fabs(delta) > .283f)) {
            p.mode = CANDIDATE;
            p.transition_ms = s.time_ms;
            p.candidate_residual = residual;
        } else {
            result.fuse_height = true;
            // Retain small coherent changes so a ramp can accumulate evidence.
            // At hover, let normal height fusion correct IMU drift instead.
            if (!translating || raw_quiet) {
                anchor(p);
            }
        }
    } else if (p.mode == CANDIDATE) {
        if (translating && std::fabs(raw_change) > .04f &&
            residual * p.candidate_residual > 0 && std::fabs(residual) > gate &&
            s.time_ms - p.transition_ms >= 40) {
            result.terrain_delta_down_m = -residual;
            p.mode = ABSORBING;
            anchor(p);
            p.window_count = 0;
            p.mean_ms = 0;
            p.sum_raw = p.sum_compensated = p.sum_time = 0;
        } else if ((quiet && std::fabs(residual) < .03f) ||
                   (raw_quiet && std::fabs(raw_change) < .03f)) {
            // A transient return disappeared; this is a fresh stable surface,
            // not a timeout or a synthetic height measurement.
            p.mode = STABLE;
            anchor(p);
            result.fuse_height = true;
        }
    } else if (p.mode == ABSORBING) {
        if (quiet) {
            p.mode = STABLE;
            anchor(p);
            result.fuse_height = true;
        } else {
            // This sample updates terrain only. Even after expiry, keep the
            // geometric estimate available without claiming height validity.
            result.terrain_delta_down_m = -residual;
            anchor(p);
        }
    } else if (quiet && std::fabs(residual) < p.gap_gate) {
        p.mode = STABLE;
        anchor(p);
        result.fuse_height = true;
    }
    if (p.mode == STABLE && raw_quiet) {
        // A stationary range must not become terrain merely because the IMU
        // reference drifts. Re-anchor this comparison only; it is not aiding.
        result.reanchor_motion = true;
        result.reanchor_time_ms = p.rate_ms;
        result.reanchor_velocity_down_mps = p.raw_rate;
    }
    result.transition = p.mode != STABLE;
    result.transition_age_ms = result.transition ? s.time_ms - p.transition_ms : 0;
    result.expired = result.transition && result.transition_age_ms >= MAX_COAST_MS;
    return result;
}
