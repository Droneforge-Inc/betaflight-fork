/* Cooperative AAPCS hard-float switch, called only from thread mode with MSP.
 * void apContextSwitch(uint32_t **fromSp, uint32_t *toSp);
 *
 * Each suspended frame is 104 bytes, indexed as uint32_t from its saved SP:
 *   0: FPSCR, 1..8: r4..r11, 9: LR, 10..25: s16..s31.
 * An initial frame has a Thumb entry address in word 9 and an initialized
 * FPSCR in word 0. Its entry must never return. Both stack tops and frames
 * are 8-byte aligned. Caller-saved registers follow the ordinary C ABI.
 *
 * Every intermediate SP is valid and aligned, so interrupts may use whichever
 * stack is current. The caller owns stack bounds and space for IRQ nesting.
 * This changes neither interrupt masks nor CONTROL/privilege/stack selection.
 */
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb
.section .text.apContextSwitch,"ax",%progbits
.align 2
.global apContextSwitch
.type apContextSwitch, %function
.thumb_func
apContextSwitch:
    vpush {d8-d15}
    vmrs r2, fpscr
    push {r2, r4-r11, lr}
    str sp, [r0]
    mov sp, r1
    pop {r2, r4-r11, lr}
    vmsr fpscr, r2
    vpop {d8-d15}
    bx lr
.size apContextSwitch, .-apContextSwitch
.section .note.GNU-stack,"",%progbits
