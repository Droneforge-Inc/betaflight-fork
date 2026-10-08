/* AP uses flash for code and a bounded allocation arena in data RAM. Reserve
 * 6 KiB of main stack for BF callers and interrupt nesting. The AP worker has
 * its own 6 KiB data-RAM stack; both need hardware high-water qualification. */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 16K
    FLASH_CONFIG (r)  : ORIGIN = 0x08004000, LENGTH = 8K
    FLASH1 (rx)       : ORIGIN = 0x08006000, LENGTH = 488K
    SYSTEM_MEMORY (r) : ORIGIN = 0x1FFF0000, LENGTH = 64K
    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 120K
    CCM (xrw)         : ORIGIN = 0x2001E000, LENGTH = 8K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", CCM)
REGION_ALIAS("FASTRAM", CCM)
REGION_ALIAS("VECTAB", CCM)
REGION_ALIAS("MOVABLE_FLASH", FLASH)

__stack_size = 0x1800;
INCLUDE "stm32_flash_g4_split.ld"
ASSERT(ORIGIN(FLASH1) + LENGTH(FLASH1) == 0x08080000, "AP firmware exceeds physical G474 flash")
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(FLASH_CONFIG), "AP vector bank overlaps configuration")
ASSERT(_efastram_data <= _heap_stack_begin && _efastram_bss <= _heap_stack_begin, "AP stack overlaps fast data")
