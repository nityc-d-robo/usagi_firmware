/**
 * @file cyphal_node.h
 * @brief Cyphal/CAN node: canard instance, RX subscriptions, FDCAN bridge.
 */

#ifndef CYPHAL_NODE_H
#define CYPHAL_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Subject IDs (fixed for now; can be made configurable via registers later). */
#define CYPHAL_SUBJECT_READINESS        3005U
#define CYPHAL_SUBJECT_SERVO_SETPOINT_0 3010U
#define CYPHAL_SUBJECT_SERVO_SETPOINT_1 3011U
#define CYPHAL_SUBJECT_SERVO_SETPOINT_2 3012U
#define CYPHAL_SUBJECT_SERVO_SETPOINT_3 3013U
#define CYPHAL_SUBJECT_PUMP_SETPOINT    3020U

/** Initialize Cyphal node (canard, dsdl, RX queue, subscriptions). Call once before scheduler. */
bool cyphal_node_init(void);

/** Start FDCAN RX (Start, ActivateNotification, register callback). Call after MX_FDCAN1_Init. */
void cyphal_node_start_fdcan(void);

/** Run one step: wait for notification or timeout, drain RX queue, canardRxAccept, dispatch. Call from task loop. */
void cyphal_node_step(void);

/** Task entry for Cyphal RX processing. */
void CyphalControlTask(void* pvParameters);

/** Number of RX frames dropped (queue full). For debug / UART logging. */
uint32_t cyphal_node_get_frames_dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* CYPHAL_NODE_H */
