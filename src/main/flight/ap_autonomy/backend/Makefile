AP_BACKEND_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BF_ROOT := $(abspath $(AP_BACKEND_ROOT)/../../../../..)
AP_VENDOR_ROOT := $(BF_ROOT)/lib/main/ardupilot
include $(AP_VENDOR_ROOT)/sources.mk
include $(AP_BACKEND_ROOT)/sources.mk

# Keep research-policy objects separate from the production-default backend.
BUILD ?= $(BF_ROOT)/obj/ap-backend-native$(if $(filter 1,$(TERRAIN)),-terrain)
CXX ?= g++
SOURCES := $(AP_VENDOR_SOURCES) $(AP_BACKEND_SOURCES)
OBJECTS := $(addprefix $(BUILD)/,$(SOURCES:$(BF_ROOT)/%.cpp=%.o))
CPPFLAGS := $(addprefix -I,$(AP_BACKEND_INCLUDES))
ifeq ($(TERRAIN),1)
CPPFLAGS += -DEK3_FEATURE_DF_TERRAIN=1
endif
ifeq ($(DIAGNOSTICS),1)
CPPFLAGS += -DUSE_DF3_BLACKBOX
endif
CXXFLAGS := -O2 -g $(AP_BACKEND_CXXFLAGS)

.PHONY: all test test-diagnostics clean
all: $(BUILD)/libap_backend.a
# Use DIAGNOSTICS=1 and a separate BUILD directory to exercise enabled capture.
# Compare the complete normal C output on each update, including runtime toggles.
test-diagnostics: $(BUILD)/test_vertical_diagnostics
	$(BUILD)/test_vertical_diagnostics off clean $(BUILD)/diag-off-clean.bin
	$(BUILD)/test_vertical_diagnostics on clean $(BUILD)/diag-on-clean.bin
	$(BUILD)/test_vertical_diagnostics toggle clean $(BUILD)/diag-toggle-clean.bin
	cmp $(BUILD)/diag-off-clean.bin $(BUILD)/diag-on-clean.bin
	cmp $(BUILD)/diag-off-clean.bin $(BUILD)/diag-toggle-clean.bin
	$(BUILD)/test_vertical_diagnostics off motor-bias $(BUILD)/diag-off-bias.bin
	$(BUILD)/test_vertical_diagnostics on motor-bias $(BUILD)/diag-on-bias.bin
	$(BUILD)/test_vertical_diagnostics toggle motor-bias $(BUILD)/diag-toggle-bias.bin
	cmp $(BUILD)/diag-off-bias.bin $(BUILD)/diag-on-bias.bin
	cmp $(BUILD)/diag-off-bias.bin $(BUILD)/diag-toggle-bias.bin

ifeq ($(TERRAIN),1)
POLICY_TESTS := $(BUILD)/test_terrain $(BUILD)/test_terrain_control $(BUILD)/test_terrain_policy
else
POLICY_TESTS := $(BUILD)/test_indoor_height
endif

test: $(BUILD)/test_backend $(BUILD)/test_heading $(BUILD)/test_vibration $(BUILD)/test_imu_timing $(BUILD)/test_vertical_integral $(POLICY_TESTS)
	$(BUILD)/test_vertical_integral
	$(BUILD)/test_imu_timing
	$(BUILD)/test_vibration
	$(BUILD)/test_backend 1 clean
	$(BUILD)/test_backend 1 startup-bias
	$(BUILD)/test_backend 1 motor-bias-down
	$(BUILD)/test_backend 1 motor-bias-up
	$(BUILD)/test_backend 1 motor-bias-0.15g
	$(BUILD)/test_backend 1 noisy
	$(BUILD)/test_backend 0
	$(BUILD)/test_heading stationary 1.570796326795
	$(BUILD)/test_heading stationary -1.570796326795
	$(BUILD)/test_heading rotate 1.570796326795
	$(BUILD)/test_heading rotate -1.570796326795
	$(BUILD)/test_heading armed-guard 1.570796326795
	$(BUILD)/test_heading omit 0
ifeq ($(TERRAIN),1)
	$(BUILD)/test_terrain_policy
	$(BUILD)/test_terrain_control
	$(BUILD)/test_terrain steps
	$(BUILD)/test_terrain slope
	$(BUILD)/test_terrain maneuver
	$(BUILD)/test_terrain maneuver-terrain
	$(BUILD)/test_terrain motor-bias
	$(BUILD)/test_terrain stale
	$(BUILD)/test_terrain invalid
	$(BUILD)/test_terrain gap-step
else
	$(BUILD)/test_indoor_height clean
	$(BUILD)/test_indoor_height disturbed
	$(BUILD)/test_indoor_height stale
	$(BUILD)/test_indoor_height invalid
endif

$(BUILD)/libap_backend.a: $(OBJECTS) $(AP_BACKEND_ROOT)/Makefile $(AP_BACKEND_ROOT)/sources.mk $(AP_VENDOR_ROOT)/sources.mk
	rm -f $@
	$(AR) rcs $@ $(OBJECTS)

$(BUILD)/test_backend: $(AP_BACKEND_ROOT)/test_backend.cpp $(BUILD)/libap_backend.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -Wl,--whole-archive $(BUILD)/libap_backend.a -Wl,--no-whole-archive $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/test_vertical_diagnostics: $(AP_BACKEND_ROOT)/test_vertical_diagnostics.cpp $(BUILD)/libap_backend.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -Wl,--whole-archive $(BUILD)/libap_backend.a -Wl,--no-whole-archive $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/test_vertical_integral: $(AP_BACKEND_ROOT)/test_vertical_integral.cpp $(BUILD)/libap_backend.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -Wl,--whole-archive $(BUILD)/libap_backend.a -Wl,--no-whole-archive $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/test_indoor_height: $(AP_BACKEND_ROOT)/test_indoor_height.cpp $(BUILD)/libap_backend.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -Wl,--whole-archive $(BUILD)/libap_backend.a -Wl,--no-whole-archive $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/test_heading: $(AP_BACKEND_ROOT)/test_heading.cpp $(BUILD)/libap_backend.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -Wl,--whole-archive $(BUILD)/libap_backend.a -Wl,--no-whole-archive $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/test_imu_timing: $(AP_BACKEND_ROOT)/test_imu_timing.cpp $(BUILD)/libap_backend.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -Wl,--whole-archive $(BUILD)/libap_backend.a -Wl,--no-whole-archive $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/test_vibration: $(AP_BACKEND_ROOT)/test_vibration.cpp $(BUILD)/libap_backend.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -Wl,--whole-archive $(BUILD)/libap_backend.a -Wl,--no-whole-archive $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/test_terrain: $(AP_BACKEND_ROOT)/test_terrain.cpp $(BUILD)/libap_backend.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -Wl,--whole-archive $(BUILD)/libap_backend.a -Wl,--no-whole-archive $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/test_terrain_control: $(AP_BACKEND_ROOT)/test_terrain_control.cpp $(BUILD)/libap_backend.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -Wl,--whole-archive $(BUILD)/libap_backend.a -Wl,--no-whole-archive $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/test_terrain_policy: $(AP_BACKEND_ROOT)/test_terrain_policy.cpp $(AP_BACKEND_ROOT)/ap_terrain.cpp $(AP_BACKEND_ROOT)/ap_terrain.h
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< $(AP_BACKEND_ROOT)/ap_terrain.cpp $(LDFLAGS) -Wl,--gc-sections -o $@

$(BUILD)/%.o: $(BF_ROOT)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(if $(filter allocator.cpp,$(notdir $<)),-DAP_BACKEND_ALLOCATOR_IMPL=1) -MMD -MP -c $< -o $@

clean:
	rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)
