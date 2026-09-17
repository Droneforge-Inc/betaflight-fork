/* Handwritten scalar Cortex-M4F kernels. No NEON, SIMD or fused operations.
 * VMUL then VSUB/VADD preserves C's two rounding steps. Six independent
 * columns amortize pointer updates and expose independent FP operations.
 * AAPCS: preserve r4-r11 and every used s16-s31; SP stays 8-byte aligned.
 * Inputs are trusted internal matrix buffers; bounds are caller's contract. */
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb
#ifdef USE_DF3_CCM
.section .ccm_code.df3_joseph,"ax",%progbits
#else
.text
#endif
.align 2

.macro FUNC name
.global \name
.type \name, %function
.thumb_func
\name:
.endm
.macro END name
.size \name, .-\name
.endm

FUNC joseph_asm_left
    cmp r2, #0
    beq 9f
    push {r4-r10, lr}
    vpush {s16-s21}
    mov r8, r2
    movs r3, #72
    mul r3, r1, r3
    ldr r4, [r0, #0]
    add r4, r3
    ldr r5, [r0, #8]
    ldr r6, [r0, #28]
    add r6, r3
    ldr r7, [r0, #4]
    add r1, r1, r1, lsl #1
    add r7, r7, r1, lsl #2
1:
    vldmia r7!, {s12-s14}
    mov r10, r5
    add r3, r5, #72
    add r12, r5, #144
    movs r9, #3
2:
    vldmia r4!, {s0-s5}
    vldmia r10!, {s6-s11}
    vmul.f32 s16, s12, s6
    vmul.f32 s17, s12, s7
    vmul.f32 s18, s12, s8
    vmul.f32 s19, s12, s9
    vmul.f32 s20, s12, s10
    vmul.f32 s21, s12, s11
    vsub.f32 s0, s0, s16
    vsub.f32 s1, s1, s17
    vsub.f32 s2, s2, s18
    vsub.f32 s3, s3, s19
    vsub.f32 s4, s4, s20
    vsub.f32 s5, s5, s21
    vldmia r3!, {s6-s11}
    vmul.f32 s16, s13, s6
    vmul.f32 s17, s13, s7
    vmul.f32 s18, s13, s8
    vmul.f32 s19, s13, s9
    vmul.f32 s20, s13, s10
    vmul.f32 s21, s13, s11
    vsub.f32 s0, s0, s16
    vsub.f32 s1, s1, s17
    vsub.f32 s2, s2, s18
    vsub.f32 s3, s3, s19
    vsub.f32 s4, s4, s20
    vsub.f32 s5, s5, s21
    vldmia r12!, {s6-s11}
    vmul.f32 s16, s14, s6
    vmul.f32 s17, s14, s7
    vmul.f32 s18, s14, s8
    vmul.f32 s19, s14, s9
    vmul.f32 s20, s14, s10
    vmul.f32 s21, s14, s11
    vsub.f32 s0, s0, s16
    vsub.f32 s1, s1, s17
    vsub.f32 s2, s2, s18
    vsub.f32 s3, s3, s19
    vsub.f32 s4, s4, s20
    vsub.f32 s5, s5, s21
    vstmia r6!, {s0-s5}
    subs r9, #1
    bne 2b
    subs r8, #1
    bne 1b
    vpop {s16-s21}
    pop {r4-r10, pc}
9:  bx lr
END joseph_asm_left

/* Sparse APH^T follows the supplied ascending support order. +0 starts
 * BOTH sums; skipping the first +0 would alter signed-zero behavior. */
.macro MIDDLE_COL k, ra, rb, rc
    vmov.f32 s0, s15
    vmov.f32 s1, s15
    add r0, r5, #(72*\k)
    add r1, r9, #(18*\k)
    ldrb r2, [r10, #\k]
    cbz r2, 4f
3:
    ldrb r3, [r1], #1
    add r12, r4, r3, lsl #2
    vldr s2, [r12]
    add r12, r0, r3, lsl #2
    vldr s3, [r12]
    vmul.f32 s2, s2, s3
    vadd.f32 s0, s0, s2
    subs r2, #1
    bne 3b
4:
    vmul.f32 s2, s12, \ra
    vadd.f32 s1, s1, s2
    vmul.f32 s2, s13, \rb
    vadd.f32 s1, s1, s2
    vmul.f32 s2, s14, \rc
    vadd.f32 s1, s1, s2
    vsub.f32 s1, s1, s0
    vstr s1, [r8, #(4*\k)]
.endm

FUNC joseph_asm_middle
    cmp r2, #0
    beq 9f
    push {r4-r11, lr}
    vpush {s16-s24}
    mov r11, r2
    movs r3, #72
    mul r3, r1, r3
    ldr r4, [r0, #28]
    add r4, r3
    ldr r5, [r0, #12]
    ldr r6, [r0, #16]
    ldr r7, [r0, #4]
    ldr r8, [r0, #8]
    add r1, r1, r1, lsl #1
    add r7, r7, r1, lsl #2
    add r8, r8, r1, lsl #2
    ldr r9, [r0, #20]
    ldr r10, [r0, #24]
    vldmia r6, {s16-s24}
    movs r0, #0
    vmov s15, r0
1:
    vldmia r7!, {s12-s14}
    MIDDLE_COL 0, s16, s19, s22
    MIDDLE_COL 1, s17, s20, s23
    MIDDLE_COL 2, s18, s21, s24
    add r4, #72
    add r8, #12
    subs r11, #1
    bne 1b
    vpop {s16-s24}
    pop {r4-r11, pc}
9:  bx lr
END joseph_asm_middle

/* Pair adjacent lower-triangle columns. Each scalar retains its original
 * multiply/add order; the four independent accumulators hide FP latency.
 * The diagonal follows the off-diagonal stores (buffers are disjoint).
 * K/B loads and lower-row loads/stores share pointer increments per pair. */
FUNC joseph_asm_sym
    cmp r2, #0
    beq 9f
    push {r3-r11, lr}
    vpush {s16-s23}
    mov r11, r1                 /* i */
    add r3, r1, r2             /* exclusive end */
    ldr r4, [r0, #28]           /* tmp base */
    ldr r5, [r0, #32]           /* out base */
    ldr r6, [r0, #4]            /* K base */
    ldr r7, [r0, #8]            /* B base */
    vmov.f32 s22, #0.5
1:
    add r0, r11, r11, lsl #1
    add r1, r7, r0, lsl #2
    add r0, r6, r0, lsl #2
    vldmia r0, {s16-s18}        /* K_i */
    vldmia r1, {s19-s21}        /* B_i */
    add r0, r11, r11, lsl #3
    add r8, r4, r0, lsl #3      /* tmp[i,0] */
    add r9, r5, r0, lsl #3      /* out[i,0] */
    add r10, r4, r11, lsl #2    /* tmp[0,i] */
    add r12, r5, r11, lsl #2    /* out[0,i] */
    mov r0, r6                 /* K_j */
    mov r1, r7                 /* B_j */
    lsrs r2, r11, #1           /* column pairs remaining */
    beq 3f
2:
    vldmia r8!, {s0-s1}
    vldr s2, [r10]
    vldr s3, [r10, #72]
    vldmia r0!, {s4-s9}
    vldmia r1!, {s10-s15}
    vmul.f32 s4, s19, s4
    vmul.f32 s7, s19, s7
    vmul.f32 s10, s10, s16
    vmul.f32 s13, s13, s16
    vadd.f32 s0, s0, s4
    vadd.f32 s1, s1, s7
    vadd.f32 s2, s2, s10
    vadd.f32 s3, s3, s13
    vmul.f32 s5, s20, s5
    vmul.f32 s8, s20, s8
    vmul.f32 s11, s11, s17
    vmul.f32 s14, s14, s17
    vadd.f32 s0, s0, s5
    vadd.f32 s1, s1, s8
    vadd.f32 s2, s2, s11
    vadd.f32 s3, s3, s14
    vmul.f32 s6, s21, s6
    vmul.f32 s9, s21, s9
    vmul.f32 s12, s12, s18
    vmul.f32 s15, s15, s18
    vadd.f32 s0, s0, s6
    vadd.f32 s1, s1, s9
    vadd.f32 s2, s2, s12
    vadd.f32 s3, s3, s15
    vadd.f32 s0, s0, s2
    vadd.f32 s1, s1, s3
    vmul.f32 s0, s0, s22
    vmul.f32 s1, s1, s22
    vstmia r9!, {s0-s1}
    vstr s0, [r12]
    vstr s1, [r12, #72]
    add r10, #144
    add r12, #144
    subs r2, #1
    bne 2b
3:
    tst r11, #1
    beq 4f
    vldmia r8!, {s0}
    vldr s1, [r10]
    vldmia r0!, {s3-s5}
    vldmia r1!, {s6-s8}
    vmul.f32 s9, s19, s3
    vmul.f32 s10, s6, s16
    vadd.f32 s0, s0, s9
    vadd.f32 s1, s1, s10
    vmul.f32 s9, s20, s4
    vmul.f32 s10, s7, s17
    vadd.f32 s0, s0, s9
    vadd.f32 s1, s1, s10
    vmul.f32 s9, s21, s5
    vmul.f32 s10, s8, s18
    vadd.f32 s0, s0, s9
    vadd.f32 s1, s1, s10
    vadd.f32 s0, s0, s1
    vmul.f32 s0, s0, s22
    vstmia r9!, {s0}
    vstr s0, [r12]
4:
    vldr s0, [r8]
    vmul.f32 s2, s19, s16
    vadd.f32 s0, s0, s2
    vmul.f32 s2, s20, s17
    vadd.f32 s0, s0, s2
    vmul.f32 s2, s21, s18
    vadd.f32 s0, s0, s2
    vstr s0, [r9]
    adds r11, #1
    cmp r11, r3
    bne 1b
    vpop {s16-s23}
    pop {r3-r11, pc}
9:  bx lr
END joseph_asm_sym
.section .note.GNU-stack,"",%progbits
