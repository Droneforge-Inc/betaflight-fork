/* Joseph covariance kernel ABI. Production ARM kernels live beside this header;
 * the simulation repository retains the C and x86-64 comparison harnesses. */
#pragma once
#include <float.h>
#include <stddef.h>
#include <stdint.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24, "Assembly requires IEEE binary32 floats");

enum { JN = 18 };
typedef struct {
    const float *P, *K;
    float *HP; /* Reused as B only after the entire left product. */
    const float *H, *R;
    const uint8_t (*columns)[JN];
    const uint8_t *count;
    float *tmp, *out;
} josephArgs;

#if defined(__arm__)
_Static_assert(sizeof(josephArgs) == 36, "Assembly requires ARM32 layout");
_Static_assert(offsetof(josephArgs, out) == 32, "Assembly layout drift");
#elif defined(__x86_64__)
_Static_assert(sizeof(josephArgs) == 72, "Assembly requires LP64 layout");
_Static_assert(offsetof(josephArgs, out) == 64, "Assembly layout drift");
#endif
#define JOSEPH_OFFSET(field, slot)                                                                                     \
    _Static_assert(offsetof(josephArgs, field) == (slot) * sizeof(void *), "Assembly field order drift")
JOSEPH_OFFSET(P, 0);
JOSEPH_OFFSET(K, 1);
JOSEPH_OFFSET(HP, 2);
JOSEPH_OFFSET(H, 3);
JOSEPH_OFFSET(R, 4);
JOSEPH_OFFSET(columns, 5);
JOSEPH_OFFSET(count, 6);
JOSEPH_OFFSET(tmp, 7);
JOSEPH_OFFSET(out, 8);
#undef JOSEPH_OFFSET

/* All buffers are disjoint except the intentional HP/B lifetime reuse.
 * Rows must be in [0,18]; complete ALL left rows before any middle rows,
 * and ALL middle rows before any symmetric rows. No concurrent mutation.
 * count[k] <= 18; columns[k] enumerates H's exact nonzero support in its
 * original summation order. Float buffers need 4-byte alignment, not 16.
 * No allocation, saturation, FMA, or change in per-element summation order. */
void joseph_c(josephArgs *a);
void joseph_asm(josephArgs *a);
void joseph_c_left(josephArgs *a, unsigned first, unsigned rows);
void joseph_c_middle(josephArgs *a, unsigned first, unsigned rows);
void joseph_c_sym(josephArgs *a, unsigned first, unsigned rows);
void joseph_asm_left(josephArgs *a, unsigned first, unsigned rows);
void joseph_asm_middle(josephArgs *a, unsigned first, unsigned rows);
void joseph_asm_sym(josephArgs *a, unsigned first, unsigned rows);
