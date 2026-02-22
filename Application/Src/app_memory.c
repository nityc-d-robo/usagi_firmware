/**
 * @file app_memory.c
 * @brief FreeRTOS heap_4 allocator wrappers for libcanard.
 */

#include "app_memory.h"
#include "canard.h"
#include "FreeRTOS.h"

static void* canard_allocate(void* const user_reference, const size_t size)
{
    (void)user_reference;
    if (size == 0U) {
        return NULL;
    }
    return pvPortMalloc(size);
}

static void canard_deallocate(void* const user_reference, const size_t size, void* const pointer)
{
    (void)user_reference;
    (void)size;
    if (pointer != NULL) {
        vPortFree(pointer);
    }
}

struct CanardMemoryResource app_memory_canard_resource(void)
{
    struct CanardMemoryResource r = {
        .user_reference = NULL,
        .deallocate     = canard_deallocate,
        .allocate       = canard_allocate,
    };
    return r;
}
