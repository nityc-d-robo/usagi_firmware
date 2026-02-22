/**
 * @file dsdl_runtime.c
 * @brief In-memory DSDL namespace for Planar, Readiness, Bit and dependencies.
 * DSDL table is generated at build time from Drivers/public_regulated_data_types (see embed_dsdl.py).
 */

#include "dsdl_runtime.h"
#include "dsdl_embedded.h"
#include "app_memory.h"
#include <dsdl.h>
#include <string.h>

static struct dsdl_t* s_dsdl;
static const struct dsdl_type_composite_t* s_cached_planar;
static const struct dsdl_type_composite_t* s_cached_readiness;
static const struct dsdl_type_composite_t* s_cached_bit;

static wkv_str_t read_cb(struct dsdl_t* self, wkv_str_t path)
{
    wkv_str_t result = { 0, NULL };
    if (path.str == NULL) {
        return result;
    }
    for (size_t i = 0; i < dsdl_embedded_count; i++) {
        size_t plen = strlen(dsdl_embedded_files[i].path);
        if (plen == path.len && memcmp(path.str, dsdl_embedded_files[i].path, path.len) == 0) {
            size_t body_len = strlen(dsdl_embedded_files[i].body);
            char* buf = (char*)app_memory_dsdl_realloc(self, NULL, body_len);
            if (buf == NULL) {
                return result;
            }
            memcpy(buf, dsdl_embedded_files[i].body, body_len);
            result.str = buf;
            result.len = body_len;
            return result;
        }
    }
    return result;
}

/* Check if path is a direct child of dir (dir ends with /; path is dir + filename with no extra /). */
static bool is_direct_child(const char* path, size_t path_len, const char* dir, size_t dir_len)
{
    if (dir_len == 0 || path_len <= dir_len) {
        return false;
    }
    if (memcmp(path, dir, dir_len) != 0) {
        return false;
    }
    const char* rest = path + dir_len;
    size_t rest_len = path_len - dir_len;
    for (size_t i = 0; i < rest_len; i++) {
        if (rest[i] == '/') {
            return false;
        }
    }
    return true;
}

static wkv_str_t* list_cb(struct dsdl_t* self, wkv_str_t path)
{
    if (path.str == NULL) {
        return NULL;
    }
    /* Count direct children. */
    size_t count = 0;
    for (size_t i = 0; i < dsdl_embedded_count; i++) {
        size_t plen = strlen(dsdl_embedded_files[i].path);
        if (is_direct_child(dsdl_embedded_files[i].path, plen, path.str, path.len)) {
            count++;
        }
    }
    if (count == 0) {
        return NULL;
    }
    wkv_str_t* arr = (wkv_str_t*)app_memory_dsdl_realloc(self, NULL, (count + 1) * sizeof(wkv_str_t));
    if (arr == NULL) {
        return NULL;
    }
    size_t idx = 0;
    for (size_t i = 0; i < dsdl_embedded_count && idx < count; i++) {
        size_t plen = strlen(dsdl_embedded_files[i].path);
        if (!is_direct_child(dsdl_embedded_files[i].path, plen, path.str, path.len)) {
            continue;
        }
        const char* filename = dsdl_embedded_files[i].path + path.len;
        size_t flen = plen - path.len;
        char* copy = (char*)app_memory_dsdl_realloc(self, NULL, flen);
        if (copy == NULL) {
            while (idx > 0) {
                app_memory_dsdl_realloc(self, (void*)arr[--idx].str, 0);
            }
            app_memory_dsdl_realloc(self, arr, 0);
            return NULL;
        }
        memcpy(copy, filename, flen);
        arr[idx].str = copy;
        arr[idx].len = flen;
        idx++;
    }
    arr[count].str = NULL;
    arr[count].len = 0;
    return arr;
}

bool dsdl_runtime_init(void)
{
    static struct dsdl_t dsdl_storage;
    dsdl_new(&dsdl_storage, app_memory_dsdl_realloc, read_cb, list_cb);
    if (!dsdl_add_namespace(&dsdl_storage, wkv_key("reg"))) {
        return false;
    }
    if (!dsdl_add_namespace(&dsdl_storage, wkv_key("uavcan"))) {
        return false;
    }
    s_dsdl = &dsdl_storage;
    s_cached_planar   = NULL;
    s_cached_readiness = NULL;
    s_cached_bit      = NULL;
    return true;
}

void dsdl_runtime_shutdown(void)
{
    s_dsdl = NULL;
    s_cached_planar   = NULL;
    s_cached_readiness = NULL;
    s_cached_bit      = NULL;
}

const struct dsdl_type_composite_t* dsdl_runtime_type_planar(void)
{
    if (s_dsdl == NULL) {
        return NULL;
    }
    if (s_cached_planar == NULL) {
        s_cached_planar = dsdl_read(s_dsdl, wkv_key("reg.udral.physics.dynamics.rotation.Planar.0.1"));
    }
    return s_cached_planar;
}

const struct dsdl_type_composite_t* dsdl_runtime_type_readiness(void)
{
    if (s_dsdl == NULL) {
        return NULL;
    }
    if (s_cached_readiness == NULL) {
        s_cached_readiness = dsdl_read(s_dsdl, wkv_key("reg.udral.service.common.Readiness.0.1"));
    }
    return s_cached_readiness;
}

const struct dsdl_type_composite_t* dsdl_runtime_type_bit(void)
{
    if (s_dsdl == NULL) {
        return NULL;
    }
    if (s_cached_bit == NULL) {
        s_cached_bit = dsdl_read(s_dsdl, wkv_key("uavcan.primitive.scalar.Bit.1.0"));
    }
    return s_cached_bit;
}
