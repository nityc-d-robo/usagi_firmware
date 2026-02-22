/**
 * @file dsdl_embedded.h
 * @brief Declarations for build-time embedded DSDL file table (see embed_dsdl.py).
 */

#ifndef DSDL_EMBEDDED_H
#define DSDL_EMBEDDED_H

#include <stddef.h>

typedef struct {
    const char* path;
    const char* body;
} dsdl_embedded_entry_t;

extern const dsdl_embedded_entry_t dsdl_embedded_files[];
extern const size_t dsdl_embedded_count;

#endif /* DSDL_EMBEDDED_H */
