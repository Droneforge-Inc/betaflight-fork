# One statically linked AP backend, with sources and support owned by this fork.
ifeq ($(filter USE_DF3,$(OPTIONS)),)
$(error USE_AP_AUTONOMY requires USE_DF3 for the reference and telemetry interface)
endif
ifneq ($(filter USE_DF3_PROFILE USE_DF3_RESUMABLE USE_DF3_BUDGETED_WORKER USE_DF3_JOSEPH_ASM USE_DF3_RESET_ASM,$(OPTIONS)),)
$(error Legacy DF3 execution options cannot be combined with USE_AP_AUTONOMY)
endif
ifneq ($(TARGET),SITL)
ifeq ($(filter STM32G474xx STM32F405xx,$(TARGET_MCU)),)
$(error Embedded AP autonomy currently targets STM32G474xx or STM32F405xx)
endif
override OPTIONS += USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW
ifeq ($(CONFIG),SPEEDYBEEF405MINI)
override OPTIONS += USE_AP_ICM42688P_4KHZ
endif
# Embedded AP uses the existing ELRS/CRSF interface. Exclude other receiver
# and telemetry protocols without removing OSD, VTX, Blackbox or motor features.
override OPTIONS += USE_SERIAL_RX USE_SERIALRX USE_SERIALRX_CRSF USE_TELEMETRY USE_TELEMETRY_CRSF
ifeq ($(TARGET_MCU),STM32G474xx)
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_g474_ap.ld
else
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f405_ap.ld
endif
# Preserve BF's explicitly speed-optimized PID/gyro/motor sources; use the
# size setting for the remaining embedded AP code.
OPTIMISE_DEFAULT := -Os
endif

AP_VENDOR_ROOT := $(ROOT)/lib/main/ardupilot
override OPTIONS += USE_AP_WORKER
ifneq ($(TARGET),SITL)
DF3_ASM_SRC := flight/ap_autonomy/ap_context_m4f.S
DF3_ASM_FLAGS := $(addprefix -D,$(OPTIONS))
endif
AP_BACKEND_ROOT := $(ROOT)/src/main/flight/ap_autonomy/backend
include $(AP_VENDOR_ROOT)/sources.mk
include $(AP_BACKEND_ROOT)/sources.mk
INCLUDE_DIRS += $(AP_BACKEND_ROOT)
AP_CPP_SOURCES := $(AP_VENDOR_SOURCES) $(AP_BACKEND_SOURCES)
AP_CPP_FLAGS = $(filter-out -fsingle-precision-constant,$(ARCH_FLAGS)) \
    $(AP_BACKEND_CXXFLAGS) -Os -g -ffp-contract=off -fstack-usage -MMD -MP \
    $(addprefix -D,$(filter USE_AP_PROFILE USE_AP_WORKER USE_DF3_BLACKBOX,$(OPTIONS))) \
    $(addprefix -I,$(AP_BACKEND_INCLUDES))
