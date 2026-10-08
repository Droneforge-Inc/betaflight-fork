AP_BACKEND_SOURCES := $(addprefix $(AP_BACKEND_ROOT)/,ap_ekf.cpp ap_control.cpp ap_terrain.cpp ap_vibration.cpp platform.cpp allocator.cpp)

AP_BACKEND_INCLUDES := $(AP_BACKEND_ROOT)/compat $(AP_BACKEND_ROOT) \
    $(AP_VENDOR_ROOT)/generated $(AP_VENDOR_ROOT)/libraries $(AP_VENDOR_ROOT)/libraries/AP_Common/missing

# config.h redirects AP allocation expressions after loading libc declarations.
# allocator.cpp itself must be built with -DAP_BACKEND_ALLOCATOR_IMPL=1.
AP_BACKEND_CXXFLAGS := -std=gnu++17 -fno-exceptions -fno-rtti -fno-threadsafe-statics \
    -fno-use-cxa-atexit -fno-strict-aliasing -fno-fast-math -ffunction-sections -fdata-sections \
    -Wno-address-of-packed-member -Wno-invalid-offsetof -Wno-psabi \
    -include $(AP_BACKEND_ROOT)/config.h
