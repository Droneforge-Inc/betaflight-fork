/* Preserve F405 flash/configuration and CCM data placement. AP needs 6 KiB
 * of main stack for BF callers and IRQ nesting, in addition to its separate
 * 6 KiB worker stack in SRAM. Neither stack's hardware high-water is qualified. */
__stack_size = 0x1800;
INCLUDE "stm32_flash_f405.ld"
ASSERT(ORIGIN(FLASH1) + LENGTH(FLASH1) == 0x08100000, "AP firmware exceeds physical F405 flash")
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(FLASH_CONFIG), "AP vector bank overlaps configuration")
ASSERT(ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG) == ORIGIN(FLASH1), "AP firmware overlaps configuration")
ASSERT(_efastram_data <= _heap_stack_begin && __fastram_bss_end__ <= _heap_stack_begin, "AP stack overlaps fast data")
