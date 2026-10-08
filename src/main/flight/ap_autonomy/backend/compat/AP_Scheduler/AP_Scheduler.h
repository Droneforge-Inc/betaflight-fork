#pragma once
#include <stdint.h>
class AP_Scheduler {
public:
    float dt = 0.0025f;
    uint32_t ticks = 0;
    uint32_t ticks32() const { return ticks; }
    float get_filtered_loop_time() const { return dt; }
    float get_loop_rate_hz() const { return 1.0f / dt; }
};
namespace AP { AP_Scheduler &scheduler(); }
