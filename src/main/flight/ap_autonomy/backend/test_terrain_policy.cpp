// Standalone: c++ -std=c++17 -O2 -Wall -Wextra -Werror ap_terrain.cpp
//             test_terrain_policy.cpp -o /tmp/ap_terrain_policy && /tmp/ap_terrain_policy
#include "ap_terrain.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>

namespace {
void require(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

struct Trial {
    ap_terrain_t policy{};
    uint32_t time_ms{1000};
    float terrain{};
    ap_terrain_result_t sample(float range, float motion_position = -4,
                               float speed = .2f, bool armed = true)
    {
        time_ms += 50;
        const ap_terrain_input_t input{time_ms, range, speed, .0001f, armed, motion_position};
        const auto result = ap_terrain_observe(policy, input);
        require(!(result.fuse_height && result.terrain_delta_down_m != 0), "one sample cannot update height and terrain");
        require(!result.expired || !result.fuse_height, "expired coast cannot fuse height");
        terrain += result.terrain_delta_down_m;
        return result;
    }
};

void steps_and_climbs()
{
    for (const float step : {-2.f, -.4f, .4f, 2.f}) {
        for (const float velocity : {-.5f, 0.f, .5f}) {
            Trial t;
            float position = -4;
            bool saw_transition = false, reacquired = false;
            for (unsigned i = 0; i < 100; ++i) {
                position += .05f * velocity;
                const float ground = i >= 30 ? step : 0;
                const auto r = t.sample(position - ground, position, .08f);
                saw_transition |= r.transition;
                if (i > 60 && r.fuse_height) {
                    reacquired = true;
                    require(std::fabs(position - ground + t.terrain - position) < .001f,
                            "step compensation must preserve real vertical motion");
                }
            }
            require(saw_transition && reacquired, "step must coast then reacquire stable height");
            require(std::fabs(t.terrain - step) < .001f, "step offset has the correct NED sign and magnitude");
        }
    }
    Trial flat;
    for (unsigned i = 0; i < 120; ++i) {
        const float time = .05f * i;
        const float position = -4 + .1f * time * time;
        const auto r = flat.sample(position, position);
        require(r.fuse_height && !r.transition && r.terrain_delta_down_m == 0,
                "constant-acceleration vertical motion on flat ground is not terrain");
    }
}

void slopes_and_small_edges()
{
    for (const float sign : {-1.f, 1.f}) {
        Trial t;
        bool expired = false, reacquired = false;
        uint32_t transition_start = 0;
        for (unsigned i = 0; i < 470; ++i) {
            // Horizontal travel 0.2 m/s over a 35% grade: 1.4 m over 4 m.
            const float ground = sign * std::fmin(1.4f, std::fmax(0.f, float(int(i) - 20) * .05f * .2f * .35f));
            const auto r = t.sample(-4 - ground);
            if (r.transition && !transition_start) {
                transition_start = t.time_ms;
            }
            if (r.transition && transition_start) {
                require(r.transition_age_ms == t.time_ms - transition_start,
                        "continued slope cannot renew the coast deadline");
            }
            expired |= r.expired;
            if (i > 450 && r.fuse_height) {
                reacquired = true;
            }
        }
        require(expired, "long ramp must report unobservable room altitude after bounded coast");
        require(reacquired, "fresh plateau can be distinguished from expired ongoing terrain");
        require(std::fabs(t.terrain - sign * 1.4f) < .02f, "small slope increments must accumulate into terrain geometry");
    }

    Trial stairs;
    for (unsigned i = 0; i < 110; ++i) {
        const float height = i < 20 ? 0 : .02f * std::fmin(30.f, float((i - 20) / 2));
        stairs.sample(-4 + height);
    }
    require(std::fabs(stairs.terrain + .6f) < .025f, "successive 2 cm edges are not each treated as vehicle motion");
}

void noise_and_hover_bias()
{
    Trial hover;
    for (unsigned i = 0; i < 600; ++i) {
        const float noise = .015f * std::sin(float(i) * 1.7f) + .005f * std::sin(float(i) * .3f);
        const auto r = hover.sample(-4 + noise, -4 + .25f * float(i) * .05f, .01f);
        require(r.fuse_height && !r.transition && hover.terrain == 0,
                "hover noise and vertical IMU bias must remain ordinary height aiding");
    }
    Trial moving_noise;
    for (unsigned i = 0; i < 300; ++i) {
        const auto r = moving_noise.sample(-4 + .02f * std::sin(float(i) * 1.7f));
        require(!r.transition && moving_noise.terrain == 0, "translation alone does not turn range noise into terrain");
    }
    Trial spike;
    bool rejected = false, recovered = false;
    for (unsigned i = 0; i < 80; ++i) {
        const auto r = spike.sample(i == 30 ? -2 : -4);
        if (i == 30) {
            rejected = !r.fuse_height && r.terrain_delta_down_m == 0;
        }
        if (i > 60) {
            recovered |= r.fuse_height;
        }
    }
    require(rejected && recovered && spike.terrain == 0, "isolated gross range spike cannot permanently shift terrain");
}

void timestamps_and_failure()
{
    Trial t;
    for (unsigned i = 0; i < 30; ++i) {
        t.sample(-4);
    }
    t.sample(-2);
    const auto confirmed = t.sample(-2);
    require(confirmed.terrain_delta_down_m < -1.9f, "test must have a confirmed step");
    ap_terrain_input_t input{t.time_ms, -2, .2f, .0001f, true, -4};
    const auto duplicate = ap_terrain_observe(t.policy, input);
    require(!duplicate.fresh && duplicate.terrain_delta_down_m == 0 && !duplicate.fuse_height,
            "duplicate timestamp must not integrate a terrain change again");
    input.time_ms -= 50;
    require(!ap_terrain_observe(t.policy, input).fresh, "out-of-order range must not reset policy history");
    input.time_ms = t.time_ms + 50;
    input.range_down_m = std::numeric_limits<float>::quiet_NaN();
    require(!ap_terrain_observe(t.policy, input).fuse_height, "invalid range must never authorize height fusion");

    t.time_ms += 500;
    auto r = t.sample(-2);
    require(!r.fuse_height && r.transition && r.terrain_delta_down_m == 0,
            "range gap must qualify new data before permitting upstream height fusion");
    for (unsigned i = 0; i < 20; ++i) {
        r = t.sample(-2);
    }
    require(r.fuse_height, "compatible stable surface must reacquire after a gap");
    t.time_ms += 500;
    for (unsigned i = 0; i < 80; ++i) {
        r = t.sample(-4);
        require(!r.fuse_height && r.terrain_delta_down_m == 0,
                "changed floor after a gap cannot force a height reset or fabricate terrain");
    }
    require(r.expired, "unqualified gap surface must expire without renewing grace");
    for (unsigned i = 0; i < 30; ++i) {
        r = t.sample(-2);
    }
    require(r.fuse_height, "return to previous geometry permits fresh stable reacquisition");
    r = t.sample(-2, -4, 0, false);
    require(r.fuse_height && !r.expired && !r.transition, "disarmed estimator reinitialization clears the policy");

    Trial wrap;
    wrap.time_ms = UINT32_MAX - 200;
    for (unsigned i = 0; i < 20; ++i) {
        require(wrap.sample(-4).fuse_height, "uint32 timestamp wrap must preserve normal fusion");
    }
}

void independent_motion()
{
    ap_terrain_t p{};
    float terrain = 0;
    bool expired = false;
    for (unsigned i = 0; i < 360; ++i) {
        const float ground = std::fmin(1.4f, std::fmax(0.f, (float(i) - 20) * .005f));
        // The classifier accepts only independent displacement. A stationary
        // motion reference must expose this ramp as a changing surface.
        ap_terrain_input_t s{1000 + i * 50, -4 + ground, .2f, .0001f, true, 0};
        const auto r = ap_terrain_observe(p, s);
        terrain += r.terrain_delta_down_m;
        expired |= r.expired;
    }
    require(expired && std::fabs(terrain + 1.4f) < .02f,
            "height-corrected velocity must not cancel independent terrain evidence");

    ap_terrain_reset(p);
    for (unsigned i = 0; i < 400; ++i) {
        const float time = float(i) * .05f;
        // Motor-on acceleration bias corrupts independent dead reckoning while
        // actual range remains unchanged during horizontal translation.
        ap_terrain_input_t s{1000 + i * 50, -4, .2f, .0001f, true, .5f * time * time};
        const auto r = ap_terrain_observe(p, s);
        require(r.fuse_height && r.terrain_delta_down_m == 0,
                "unchanged raw range must not become terrain due to inertial bias");
    }

    ap_terrain_reset(p);
    bool ambiguous = false;
    expired = false;
    for (unsigned i = 0; i < 300; ++i) {
        const float ground = std::fmin(1.0f, float(i) * .0035f);
        ap_terrain_input_t s{1000 + i * 50, -4 + ground, .2f, .046f, true, 0};
        const auto r = ap_terrain_observe(p, s);
        ambiguous |= r.transition;
        expired |= r.expired;
        require(r.terrain_delta_down_m == 0,
                "large inertial uncertainty cannot justify a fabricated terrain offset");
        if (r.transition) {
            require(!r.fuse_height, "uncertain ramp cannot continue as absolute height aiding");
        }
    }
    require(ambiguous && expired, "uncertainty must not suppress bounded ambiguity detection");

    ap_terrain_reset(p);
    for (unsigned i = 0; i < 120; ++i) {
        const float time = float(i) * .05f;
        const float position = -4 + .2f * time;
        ap_terrain_input_t s{1000 + i * 50, position, .2f, .046f, true, position};
        const auto r = ap_terrain_observe(p, s);
        require(r.fuse_height && !r.transition, "uncertainty alone must not reject genuine vertical motion");
    }
}
} // namespace

int main()
{
    steps_and_climbs();
    slopes_and_small_edges();
    noise_and_hover_bias();
    timestamps_and_failure();
    independent_motion();
    std::puts("terrain policy: steps, real motion, slopes, expiry, reacquisition, noise and timestamps passed");
}
