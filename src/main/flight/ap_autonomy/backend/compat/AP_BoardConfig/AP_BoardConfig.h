#pragma once
#include <AP_HAL/AP_HAL.h>
class AP_BoardConfig {
public:
    static void config_error(const char *message) { AP_HAL::panic("%s", message); }
    static void allocation_error(const char *message) { AP_HAL::panic("AP allocation: %s", message); }
};
