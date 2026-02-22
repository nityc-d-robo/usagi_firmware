/**
 * @file dsdl_runtime.h
 * @brief Runtime DSDL parser with in-memory namespace (Planar, Readiness, Bit).
 */

#ifndef DSDL_RUNTIME_H
#define DSDL_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>

struct dsdl_type_composite_t;

/** Initialize DSDL parser and add in-memory namespaces "reg" and "uavcan". Call once. */
bool dsdl_runtime_init(void);

/** Release resources. */
void dsdl_runtime_shutdown(void);

/** Get type descriptor for reg.udral.physics.dynamics.rotation.Planar.0.1. */
const struct dsdl_type_composite_t* dsdl_runtime_type_planar(void);

/** Get type descriptor for reg.udral.service.common.Readiness.0.1. */
const struct dsdl_type_composite_t* dsdl_runtime_type_readiness(void);

/** Get type descriptor for uavcan.primitive.scalar.Bit.1.0. */
const struct dsdl_type_composite_t* dsdl_runtime_type_bit(void);

#ifdef __cplusplus
}
#endif

#endif /* DSDL_RUNTIME_H */
