/**
 * @file app_memory.h
 * @brief FreeRTOS heap_4 wrappers for libcanard (allocate/deallocate).
 */

#ifndef APP_MEMORY_H
#define APP_MEMORY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

struct CanardMemoryResource;

/** libcanard v4 memory resource (allocate + deallocate, backed by heap_4). */
struct CanardMemoryResource app_memory_canard_resource(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MEMORY_H */
