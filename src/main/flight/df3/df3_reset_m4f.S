/* Exact binary32 reset of the fixed 18-state DF3 covariance. Input P is
 * symmetric, J is 3x3 row-major, tmp is the existing 18x18 workspace.
 * Every retained product has a separate VMUL and VADD in k=0,1,2 order;
 * the accumulator starts at +0 and zero coefficients are skipped.
 * Six independent sums keep the partial rows in caller-saved FP registers.
 * AAPCS: r4-r11 preserved, s16-s31 untouched, stack 8-byte aligned.
 * No calls, allocation, persistent storage, or fused arithmetic. */
#if defined(USE_DF3_MULTIRATE) && defined(USE_DF3_RESET_ASM)
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb
#ifdef USE_DF3_CCM
.section .ccm_code.df3_reset,"ax",%progbits
#else
.section .text.df3ResetCovarianceM4f,"ax",%progbits
#endif
.align 2
.global df3ResetCovarianceM4f
.type df3ResetCovarianceM4f,%function
.thumb_func
df3ResetCovarianceM4f:
    push {r4-r11,lr}
    sub sp,#4
    str r1,[sp]
    mov r4,r0
    mov r5,r2
    mov r6,r1
    add r7,r5,#648
    movs r8,#3
    movs r0,#0
    vmov s15,r0
.Lleft_row:
    vldmia r6!,{s12-s14}
    movs r10,#0
    vcmp.f32 s12,#0
    vmrs APSR_nzcv,fpscr
    it ne
    orrne r10,r10,#1
    vcmp.f32 s13,#0
    vmrs APSR_nzcv,fpscr
    it ne
    orrne r10,r10,#2
    vcmp.f32 s14,#0
    vmrs APSR_nzcv,fpscr
    it ne
    orrne r10,r10,#4
    add r0,r4,#648
    add r1,r0,#72
    add r2,r0,#144
    movs r11,#3
    cmp r10,#7
    beq .Lleft_dense_chunk
.Lleft_chunk:
    vmov.f32 s0,s15
    vmov.f32 s1,s15
    vmov.f32 s2,s15
    vmov.f32 s3,s15
    vmov.f32 s4,s15
    vmov.f32 s5,s15
.macro LEFT_TERM bit,ptr,coefficient
    tst r10,#\bit
    beq .Lleft_skip\@
    vldmia \ptr,{s6-s11}
    vmul.f32 s6,\coefficient,s6
    vmul.f32 s7,\coefficient,s7
    vmul.f32 s8,\coefficient,s8
    vmul.f32 s9,\coefficient,s9
    vmul.f32 s10,\coefficient,s10
    vmul.f32 s11,\coefficient,s11
    vadd.f32 s0,s0,s6
    vadd.f32 s1,s1,s7
    vadd.f32 s2,s2,s8
    vadd.f32 s3,s3,s9
    vadd.f32 s4,s4,s10
    vadd.f32 s5,s5,s11
.Lleft_skip\@:
    add \ptr,\ptr,#24
.endm
    LEFT_TERM 1,r0,s12
    LEFT_TERM 2,r1,s13
    LEFT_TERM 4,r2,s14
    vstmia r7!,{s0-s5}
    subs r11,#1
    bne .Lleft_chunk
    b .Lleft_row_done
.Lleft_dense_chunk:
    vmov.f32 s0,s15
    vmov.f32 s1,s15
    vmov.f32 s2,s15
    vmov.f32 s3,s15
    vmov.f32 s4,s15
    vmov.f32 s5,s15
.macro LEFT_DENSE ptr,coefficient
    vldmia \ptr!,{s6-s11}
    vmul.f32 s6,\coefficient,s6
    vmul.f32 s7,\coefficient,s7
    vmul.f32 s8,\coefficient,s8
    vmul.f32 s9,\coefficient,s9
    vmul.f32 s10,\coefficient,s10
    vmul.f32 s11,\coefficient,s11
    vadd.f32 s0,s0,s6
    vadd.f32 s1,s1,s7
    vadd.f32 s2,s2,s8
    vadd.f32 s3,s3,s9
    vadd.f32 s4,s4,s10
    vadd.f32 s5,s5,s11
.endm
    LEFT_DENSE r0,s12
    LEFT_DENSE r1,s13
    LEFT_DENSE r2,s14
    vstmia r7!,{s0-s5}
    subs r11,#1
    bne .Lleft_dense_chunk
.Lleft_row_done:
    subs r8,#1
    bne .Lleft_row
    /* Preserve unchanged lower cross block before any P writes. */
    add r6,r4,#900
    add r7,r5,#900
    movs r8,#6
.Lcopy_lower:
    ldmia r6,{r0-r2}
    stmia r7,{r0-r2}
    add r6,#72
    add r7,#72
    subs r8,#1
    bne .Lcopy_lower
    /* First nine cross columns: copy and reflect the three changed rows. */
    add r6,r5,#648
    add r7,r4,#648
    add r8,r4,#36
    movs r9,#3
.Lmirror_row:
    mov r0,r6
    mov r1,r7
    mov r2,r8
    movs r3,#9
.Lmirror_column:
    ldr r10,[r0],#4
    str r10,[r1],#4
    str r10,[r2],#72
    subs r3,#1
    bne .Lmirror_column
    add r6,#72
    add r7,#72
    add r8,#4
    subs r9,#1
    bne .Lmirror_row
    /* Reload original J, saved in the stack alignment word. */
    ldr r6,[sp]
    movs r8,#9
.Lright_column:
    vldmia r6!,{s12-s14}
    movs r10,#0
    vcmp.f32 s12,#0
    vmrs APSR_nzcv,fpscr
    it ne
    orrne r10,r10,#1
    vcmp.f32 s13,#0
    vmrs APSR_nzcv,fpscr
    it ne
    orrne r10,r10,#2
    vcmp.f32 s14,#0
    vmrs APSR_nzcv,fpscr
    it ne
    orrne r10,r10,#4
    movs r0,#72
    mul r0,r8,r0
    add r1,r5,r0
    add r1,#36
    add r2,r4,r0
    add r2,r2,r8,lsl #2
    mov r3,r2
    rsb r9,r8,#18
    cmp r10,#7
    beq .Lright_dense_row
.Lright_row:
    vldmia r1,{s0-s2}
    vmov.f32 s3,s15
.macro RIGHT_TERM bit,value,coefficient
    tst r10,#\bit
    beq .Lright_skip\@
    vmul.f32 \value,\value,\coefficient
    vadd.f32 s3,s3,\value
.Lright_skip\@:
.endm
    RIGHT_TERM 1,s0,s12
    RIGHT_TERM 2,s1,s13
    RIGHT_TERM 4,s2,s14
    vstr s3,[r2]
    vstr s3,[r3]
    add r1,#72
    add r2,#72
    add r3,#4
    subs r9,#1
    bne .Lright_row
    b .Lright_column_done
.Lright_dense_row:
    vldmia r1,{s0-s2}
    vmul.f32 s0,s0,s12
    vmul.f32 s1,s1,s13
    vmul.f32 s2,s2,s14
    vadd.f32 s3,s15,s0
    vadd.f32 s3,s3,s1
    vadd.f32 s3,s3,s2
    vstr s3,[r2]
    vstr s3,[r3]
    add r1,#72
    add r2,#72
    add r3,#4
    subs r9,#1
    bne .Lright_dense_row
.Lright_column_done:
    adds r8,#1
    cmp r8,#12
    bne .Lright_column
    add sp,#4
    pop {r4-r11,pc}
.size df3ResetCovarianceM4f,.-df3ResetCovarianceM4f
#endif
