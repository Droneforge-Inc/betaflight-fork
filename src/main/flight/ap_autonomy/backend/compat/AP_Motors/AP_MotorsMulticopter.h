#pragma once
#include "AP_Motors.h"
class AP_MotorsMulticopter : public AP_Motors {
public:
    float get_throttle_thrust_max() const { return 1; }
};
