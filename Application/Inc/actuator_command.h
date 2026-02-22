/**
 * @file actuator_command.h
 * @brief Decode Cyphal transfers (Planar, Bit, Readiness) and maintain command state with timeout.
 */

#ifndef ACTUATOR_COMMAND_H
#define ACTUATOR_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

struct CanardRxTransfer;
struct CanardRxSubscription;

/** Called when a full transfer is received. Dispatches by subject and updates internal state. */
void actuator_command_on_transfer(const struct CanardRxTransfer* transfer,
                                  const struct CanardRxSubscription* subscription);

/** Apply current command state to PWM (call periodically from task). Handles CONTROL_TIMEOUT. */
void actuator_command_apply(void);

/** Initialize command state (call once). */
void actuator_command_init(void);

/** Read debug counters for validation / UART logging. */
void actuator_command_get_stats(uint32_t* decode_errors, uint32_t* timeout_count);

#ifdef __cplusplus
}
#endif

#endif /* ACTUATOR_COMMAND_H */
