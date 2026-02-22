/**
 * @file app_memory.c
 * @brief FreeRTOS heap_4: libcanard = allocate/deallocate; dsdl.c = realloc-compatible.
 */

#include "app_memory.h"
#include "canard.h"
#include "FreeRTOS.h"
#include <string.h>

/* --- libcanard (2 functions only) --- */

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

/* --- dsdl.c realloc-compatible (heap_4 has no realloc; we store size prefix) --- */

#define DSDL_PREFIX_SIZE (sizeof(size_t))

void* app_memory_dsdl_realloc(struct dsdl_t* self, void* pointer, size_t new_size)
{
    (void)self;
    if (new_size == 0U) {
        if (pointer != NULL) {
            void* block = (char*)pointer - DSDL_PREFIX_SIZE;
            vPortFree(block);
        }
        return NULL;
    }
    if (pointer == NULL) {
        void* block = pvPortMalloc(DSDL_PREFIX_SIZE + new_size);
        if (block == NULL) {
            return NULL;
        }
        *(size_t*)block = new_size;
        return (char*)block + DSDL_PREFIX_SIZE;
    }
    {
        void* block   = (char*)pointer - DSDL_PREFIX_SIZE;
        size_t old_sz = *(size_t*)block;
        void* new_block = pvPortMalloc(DSDL_PREFIX_SIZE + new_size);
        if (new_block == NULL) {
            return NULL;
        }
        *(size_t*)new_block = new_size;
        size_t copy_sz = old_sz < new_size ? old_sz : new_size;
        (void)memcpy((char*)new_block + DSDL_PREFIX_SIZE, pointer, copy_sz);
        vPortFree(block);
        return (char*)new_block + DSDL_PREFIX_SIZE;
    }
}
