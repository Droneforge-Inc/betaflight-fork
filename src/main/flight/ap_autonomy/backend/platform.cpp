#include "runtime.h"

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/print_vprintf.h>
#include <cstdarg>
#include <new>

namespace {
// AP_HAL/Util.cpp's bounded stream contract: count the full result while only
// writing the caller's available space. The upstream formatter supports floats
// without importing newlib's separate heap or optional printf-float runtime.
class BufferPrinter : public AP_HAL::BetterStream {
public:
    BufferPrinter(char *buffer, size_t capacity) : text(buffer), size(capacity) {}
    size_t write(uint8_t character) override {
        if (used < size) {
            text[used] = character;
        }
        ++used;
        return 1;
    }
    size_t write(const uint8_t *buffer, size_t length) override {
        for (size_t i = 0; i < length; ++i) {
            write(buffer[i]);
        }
        return length;
    }
    uint32_t available() override { return 0; }
    bool read(uint8_t &) override { return false; }
    uint32_t txspace() override { return 0; }
    bool discard_input() override { return false; }
    size_t used = 0;
private:
    char *text;
    size_t size;
};

int format_message(char *text, size_t size, const char *format, va_list args)
{
    BufferPrinter output(text, size ? size - 1 : 0);
    print_vprintf(&output, format, args);
    if (size) {
        text[output.used < size ? output.used : size - 1] = '\0';
    }
    return int(output.used);
}
}

static uint64_t snapshot_time_us;
void ap_backend_set_time_us(uint64_t time_us) { snapshot_time_us = time_us; }

namespace AP_HAL {
void init() {}
uint64_t micros64() { return snapshot_time_us; }
uint32_t micros() { return uint32_t(snapshot_time_us); }
uint32_t millis() { return uint32_t(snapshot_time_us / 1000); }
uint64_t millis64() { return snapshot_time_us / 1000; }
void panic(const char *fmt, ...) {
    char reason[96];
    va_list args;
    va_start(args, fmt);
    format_message(reason, sizeof(reason), fmt, args);
    va_end(args);
    ap_runtime_panic(reason);
}
int Util::vsnprintf(char *str, size_t size, const char *fmt, va_list args) {
    return format_message(str, size, fmt, args);
}
int Util::snprintf(char *str, size_t size, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    const int result = format_message(str, size, fmt, args);
    va_end(args);
    return result;
}
void Util::set_soft_armed(bool armed) {
    soft_armed = armed;
    last_armed_change_ms = millis();
}
}

class BetaflightUtil : public AP_HAL::Util {
public:
    void set_hw_rtc(uint64_t) override {}
    uint64_t get_hw_rtc() const override { return 0; }
    uint32_t available_memory() override { return ap_backend_available_memory(); }
    void *malloc_type(size_t size, Memory_Type) override { return ap_backend_malloc(size); }
    void free_type(void *pointer, size_t, Memory_Type) override { ap_backend_free(pointer); }
};
class BetaflightHAL : public AP_HAL::HAL {
public:
    explicit BetaflightHAL(BetaflightUtil *util) : AP_HAL::HAL(
        nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr,
        util, nullptr, nullptr, nullptr) {}
    void run(int, char *const[], Callbacks *) const override {}
};
alignas(BetaflightUtil) static unsigned char util_storage[sizeof(BetaflightUtil)];
alignas(BetaflightHAL) static unsigned char hal_storage[sizeof(BetaflightHAL)];
const AP_HAL::HAL &hal = *reinterpret_cast<BetaflightHAL *>(hal_storage);

#if defined(__arm__)
extern "C" {
extern void (*__init_array_start[])();
extern void (*__init_array_end[])();
}
#endif

void ap_backend_runtime_init()
{
    static bool initialized;
    if (!initialized) {
        // BF's G4 startup intentionally skips the C++ runtime. Run the linked
        // AP object constructors once before any parameter or estimator access.
        // Host processes have already done this in their normal startup.
#if defined(__arm__)
        for (auto constructor = __init_array_start; constructor != __init_array_end; ++constructor) {
            (*constructor)();
        }
#endif
        auto *util = new (util_storage) BetaflightUtil;
        new (hal_storage) BetaflightHAL(util);
        initialized = true;
    }
}

// BF IMU samples already use the vehicle frame and a colocated origin. The
// standalone DAL only reads this object's zero-valued IMU lever arm.
AP_InertialSensor::AP_InertialSensor() {}
void AP_InertialSensor::_acal_save_calibrations() { AP_HAL::panic("BF owns IMU calibration"); }
void AP_InertialSensor::_acal_event_failure() { AP_HAL::panic("BF owns IMU calibration"); }
namespace AP {
AP_InertialSensor &ins() {
    alignas(AP_InertialSensor) static unsigned char storage[sizeof(AP_InertialSensor)];
    static AP_InertialSensor *inertial_sensor;
    if (!inertial_sensor) {
        inertial_sensor = new (storage) AP_InertialSensor;
    }
    return *inertial_sensor;
}
}

extern "C" void __cxa_pure_virtual() { ap_runtime_panic("AP pure virtual call"); }
void operator delete(void *pointer, size_t) noexcept { ap_backend_free(pointer); }
void operator delete[](void *pointer, size_t) noexcept { ap_backend_free(pointer); }

// BF owns persistent configuration. Retain upstream scalar layouts and evaluate
// constructor defaults, including pointer defaults used by the PID constructors.
void AP_Param::set_value(ap_var_type type, void *ptr, float value) {
    switch (type) {
    case AP_PARAM_INT8: static_cast<AP_Int8 *>(ptr)->set(int8_t(value)); break;
    case AP_PARAM_INT16: static_cast<AP_Int16 *>(ptr)->set(int16_t(value)); break;
    case AP_PARAM_INT32: static_cast<AP_Int32 *>(ptr)->set(int32_t(value)); break;
    case AP_PARAM_FLOAT: static_cast<AP_Float *>(ptr)->set(value); break;
    case AP_PARAM_VECTOR3F: static_cast<AP_Vector3f *>(ptr)->set(Vector3f(value, value, value)); break;
    default: AP_HAL::panic("Unsupported AP parameter type %u", unsigned(type));
    }
}

void AP_Param::setup_object_defaults(const void *object, const GroupInfo *group) {
    for (const GroupInfo *entry = group; entry->type != AP_PARAM_NONE; ++entry) {
        if (entry->type > AP_PARAM_VECTOR3F) {
            continue;
        }
        void *ptr = reinterpret_cast<uint8_t *>(const_cast<void *>(object)) + entry->offset;
        const float value = entry->flags & AP_PARAM_FLAG_DEFAULT_POINTER
            ? *reinterpret_cast<const float *>(reinterpret_cast<const uint8_t *>(ptr) - entry->def_value_offset)
            : entry->def_value;
        set_value(ap_var_type(entry->type), ptr, value);
    }
}

bool AP_Param::set_object_value(const void *object, const GroupInfo *group, const char *name, float value) {
    for (const GroupInfo *entry = group; entry->type != AP_PARAM_NONE; ++entry) {
        if (entry->type <= AP_PARAM_FLOAT && strcmp(entry->name, name) == 0) {
            set_value(ap_var_type(entry->type), reinterpret_cast<uint8_t *>(const_cast<void *>(object)) + entry->offset, value);
            return true;
        }
    }
    return false;
}

bool AP_Param::configured() const { return true; }
bool AP_Param::configured_in_storage() const { return true; }
bool AP_Param::load() { return false; }
void AP_Param::save(bool) {}
void AP_Param::load_object_from_eeprom(const void *, const GroupInfo *) {}

template<typename T, ap_var_type PT> void AP_ParamTBase<T, PT>::set_and_default(const T &v) { set(v); }
template<typename T, ap_var_type PT> void AP_ParamTBase<T, PT>::set_default(const T &v) { set(v); }
template<typename T, ap_var_type PT> void AP_ParamTBase<T, PT>::set_and_save(const T &v) { set(v); }
template<typename T, ap_var_type PT> void AP_ParamTBase<T, PT>::set_and_notify(const T &v) { set(v); }
template class AP_ParamTBase<int8_t, AP_PARAM_INT8>;
template class AP_ParamTBase<int16_t, AP_PARAM_INT16>;
template class AP_ParamTBase<int32_t, AP_PARAM_INT32>;
template class AP_ParamTBase<float, AP_PARAM_FLOAT>;
