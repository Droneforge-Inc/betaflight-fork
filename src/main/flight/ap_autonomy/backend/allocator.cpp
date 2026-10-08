#include "runtime.h"

#include <cstddef>
#include <cstring>
#include <limits>
#include <new>

namespace {
constexpr size_t arena_size = 12 * 1024;
constexpr size_t alignment = alignof(std::max_align_t);
struct alignas(std::max_align_t) Block {
    size_t size;
    bool used;
};
alignas(std::max_align_t) unsigned char arena[arena_size];
bool initialized;

Block *next(Block *block)
{
    auto *address = reinterpret_cast<unsigned char *>(block) + sizeof(Block) + block->size;
    return address < arena + arena_size ? reinterpret_cast<Block *>(address) : nullptr;
}

Block *first()
{
    auto *block = reinterpret_cast<Block *>(arena);
    if (!initialized) {
        new (block) Block{arena_size - sizeof(Block), false};
        initialized = true;
    }
    return block;
}
}

void *ap_backend_malloc(size_t size)
{
    if (size > arena_size - sizeof(Block)) {
        return nullptr;
    }
    size = (size ? size : 1) + alignment - 1;
    size -= size % alignment;
    for (Block *block = first(); block; block = next(block)) {
        if (block->used || block->size < size) {
            continue;
        }
        if (block->size >= size + sizeof(Block) + alignment) {
            auto *remainder = reinterpret_cast<Block *>(reinterpret_cast<unsigned char *>(block + 1) + size);
            new (remainder) Block{block->size - size - sizeof(Block), false};
            block->size = size;
        }
        block->used = true;
        void *result = block + 1;
        std::memset(result, 0, block->size);
        return result;
    }
    return nullptr;
}

void *ap_backend_calloc(size_t count, size_t size)
{
    if (size && count > std::numeric_limits<size_t>::max() / size) {
        return nullptr;
    }
    return ap_backend_malloc(count * size);
}

void ap_backend_free(void *pointer)
{
    if (!pointer) {
        return;
    }
    // Validate against block starts rather than dereferencing an arbitrary
    // pointer. A corrupted owner must fail explicitly instead of damaging BF.
    Block *block = first();
    while (block && block + 1 != pointer) {
        block = next(block);
    }
    if (!block || !block->used) {
        ap_runtime_panic("AP allocator invalid free");
    }
    block->used = false;
    for (block = first(); block;) {
        Block *following = next(block);
        if (!block->used && following && !following->used) {
            block->size += sizeof(Block) + following->size;
        } else {
            block = following;
        }
    }
}

uint32_t ap_backend_available_memory()
{
    // EKF's core allocation requires contiguous memory; report the largest
    // available allocation rather than a potentially fragmented sum.
    size_t available = 0;
    for (Block *block = first(); block; block = next(block)) {
        if (!block->used && block->size > available) {
            available = block->size;
        }
    }
    return available;
}
