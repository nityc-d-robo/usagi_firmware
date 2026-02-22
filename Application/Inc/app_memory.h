/**
 * @file app_memory.h
 * @brief FreeRTOS heap_4 wrappers for libcanard (allocate/deallocate) and dsdl.c (realloc).
 */

#ifndef APP_MEMORY_H
#define APP_MEMORY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

struct CanardMemoryResource;
struct dsdl_t;

/** libcanard v4 memory resource (allocate + deallocate only). Use from task context only. */
struct CanardMemoryResource app_memory_canard_resource(void);

/** realloc-compatible callback for dsdl_new() (heap_4: no realloc; uses size prefix). */
void* app_memory_dsdl_realloc(struct dsdl_t* self, void* pointer, size_t new_size);

#ifdef __cplusplus
}
#endif

#endif /* APP_MEMORY_H */
