/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32G474 (Category 3) device with
**                512KByte FLASH and 96KByte SRAM and 32KByte CCM SRAM
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 10K
    FLASH_CONFIG (r)  : ORIGIN = 0x08004000, LENGTH = 8K
    FLASH1 (rx)       : ORIGIN = 0x08006000, LENGTH = 496K

    SYSTEM_MEMORY (r) : ORIGIN = 0x1FFF0000, LENGTH = 64K
/* DF3-only placement. The first 12 KiB of physical CCM remain ordinary RAM.
 * The following 16 KiB hold code through the I-Code alias at 0x10003000.
 * The final 4 KiB retain the stock vector-table and stack addresses.
 */
    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 108K
    CCMCODE (rx)      : ORIGIN = 0x10003000, LENGTH = 16K
    CCM (xrw)         : ORIGIN = 0x2001F000, LENGTH = 4K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", CCM)
REGION_ALIAS("FASTRAM", CCM)
REGION_ALIAS("VECTAB", CCM)

/* Put various bits and bobs of data into the free space after the vector table in sector 0 to save flash space. */

REGION_ALIAS("MOVABLE_FLASH", FLASH)

INCLUDE "stm32_flash_g4_split.ld"

/* 0x10003000 and 0x2001B000 are aliases of the same physical CCM. */
SECTIONS
{
  ccm_code = LOADADDR(.ccm_code);
  .ccm_code :
  {
    . = ALIGN(4);
    ccm_code_start = .;
    *(.ccm_code)
    *(.ccm_code.*)
    . = ALIGN(4);
    ccm_code_end = .;
  } >CCMCODE AT>FLASH1
}
ASSERT(ORIGIN(RAM)+LENGTH(RAM) <= ORIGIN(CCMCODE)+0x10018000, "DF3 CCM overlaps data RAM")
ASSERT(ORIGIN(CCMCODE)+LENGTH(CCMCODE)+0x10018000 <= ORIGIN(CCM), "DF3 CCM overlaps vector/stack")
ASSERT(SIZEOF(.ccm_code) <= LENGTH(CCMCODE), "DF3 hot code exceeds reserved CCM")
ASSERT(ccm_code_start >= ORIGIN(CCMCODE) && ccm_code_end <= ORIGIN(CCMCODE)+LENGTH(CCMCODE), "DF3 CCM bounds")
