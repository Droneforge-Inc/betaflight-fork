# Production: USE_DF3. Optional diagnostics: USE_DF3_PROFILE, USE_DF3_BLACKBOX.
# Kernel source/assembler flags below are derived from the target, not user knobs.
DF3_ASM_SRC :=
DF3_ASM_FLAGS :=
DF3_LINK_DEPS :=

ifneq ($(filter USE_DF3,$(OPTIONS)),)
ifneq ($(filter-out DF3_FUSION_HZ=30,$(filter DF3_FUSION_HZ=%,$(OPTIONS))),)
$(error Production DF3 uses 30 Hz; comparison builds belong in the research harness)
endif
ifneq ($(filter USE_DF3_SCHED_BENCH,$(OPTIONS)),)
$(error DF3 scheduler benchmarks belong in the research harness; use USE_DF3_PROFILE for production diagnostics)
endif
# The runtime contract owns these details; tolerate redundant legacy spellings.
# Recompute backend flags when make recursively changes the target.
override OPTIONS := $(filter-out USE_DF3_RESUMABLE USE_DF3_MULTIRATE USE_DF3_BUDGETED_WORKER \
                    USE_DF3_BUDGET_CYCLES USE_DF3_PROFILE_AUTOSTART DF3_FUSION_HZ=30 \
                    USE_DF3_JOSEPH_ASM USE_DF3_RESET_ASM,$(OPTIONS))
CFLAGS += -include $(ROOT)/src/main/flight/df3/df3_runtime.h
DF3_ASM_FLAGS += -include $(ROOT)/src/main/flight/df3/df3_runtime.h

# SITL enables these sensors in target.h and measures work using its host clock.
ifneq ($(TARGET),SITL)
override OPTIONS += USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW

# M4/M7 hard-float targets use the scalar ARM kernels; other targets use C.
ifneq ($(and $(filter -mcpu=cortex-m4 -mcpu=cortex-m7,$(ARCH_FLAGS)),$(filter -mfloat-abi=hard,$(ARCH_FLAGS))),)
override OPTIONS += USE_DF3_JOSEPH_ASM USE_DF3_RESET_ASM
DF3_ASM_SRC += flight/df3/joseph_m4f.S flight/df3/df3_reset_m4f.S
endif
endif

# Keep the existing G474 executable-CCM layout; other MCUs use normal memory.
ifeq ($(TARGET_MCU),STM32G474xx)
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_g474_df3.ld
DEVICE_FLAGS += -DUSE_DF3_CCM -DUSE_CCM_CODE
DF3_ASM_FLAGS += -DUSE_DF3_CCM
endif

override OPTIONS := $(sort $(OPTIONS))
DF3_ASM_FLAGS += $(addprefix -D,$(OPTIONS))
endif
