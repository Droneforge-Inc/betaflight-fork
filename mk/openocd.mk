OPENOCD ?= openocd
OPENOCD_IF ?= interface/stlink-v2.cfg

ifeq ($(TARGET_MCU_FAMILY),STM32G4)
OPENOCD_CFG := target/stm32g4x.cfg
endif

ifneq ($(OPENOCD_CFG),)
OPENOCD_COMMAND = $(OPENOCD) -f $(OPENOCD_IF) -f $(OPENOCD_CFG)
endif
