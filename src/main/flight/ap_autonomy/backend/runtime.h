#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Betaflight supplies these hooks. Calls stay on the cooperative scheduler thread.
uint64_t ap_runtime_time_us(void);
__attribute__((noreturn)) void ap_runtime_panic(const char *reason);

// Only the AP translation units route allocations here. The fixed arena is
// zero-filled and never borrows Betaflight's stack or peripheral DMA memory.
void *ap_backend_malloc(size_t size);
void *ap_backend_calloc(size_t count, size_t size);
void ap_backend_free(void *pointer);
uint32_t ap_backend_available_memory(void);
void ap_backend_runtime_init(void);
// AP clocks follow the accepted sensor/control snapshot throughout a transaction,
// including time spent suspended by the cooperative worker.
void ap_backend_set_time_us(uint64_t time_us);

#ifdef __cplusplus
}
#endif
