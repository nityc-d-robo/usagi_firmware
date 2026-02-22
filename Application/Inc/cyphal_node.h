/**
 * @file cyphal_node.h
 * @brief Cyphal/CAN node: canard instance, RX subscriptions, FDCAN bridge, typed TX publish API.
 *
 * Subject IDs and their corresponding nunavut-generated types are co-located here so that callers
 * only need to include this header, fill in a struct, and call a publish function.
 *
 *   Example (publish Planar setpoint):
 *
 *     reg_udral_physics_dynamics_rotation_Planar_0_1 msg;
 *     reg_udral_physics_dynamics_rotation_Planar_0_1_initialize_(&msg);
 *     msg.kinematics.angular_position.radian = 0.5f;
 *     cyphal_node_publish_planar(CYPHAL_SUBJECT_SERVO_FEEDBACK_0, &msg);
 */

#ifndef CYPHAL_NODE_H
#define CYPHAL_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "canard.h"

/* nunavut-generated message types used by the publish API */
#include <reg/udral/physics/dynamics/rotation/Planar_0_1.h>
#include <reg/udral/service/common/Readiness_0_1.h>
#include <uavcan/primitive/scalar/Bit_1_0.h>

/* ---------------------------------------------------------------------------
 * Subject IDs
 *   RX (subscribed):  READINESS, SERVO_SETPOINT_*, PUMP_SETPOINT
 *   TX (published):   extend as needed; add a typed publish function below.
 * --------------------------------------------------------------------------- */
#define CYPHAL_SUBJECT_READINESS        3005U  /* reg.udral.service.common.Readiness.0.1  */
#define CYPHAL_SUBJECT_SERVO_SETPOINT_0 3010U  /* reg.udral.physics.dynamics.rotation.Planar.0.1 */
#define CYPHAL_SUBJECT_SERVO_SETPOINT_1 3011U  /* reg.udral.physics.dynamics.rotation.Planar.0.1 */
#define CYPHAL_SUBJECT_SERVO_SETPOINT_2 3012U  /* reg.udral.physics.dynamics.rotation.Planar.0.1 */
#define CYPHAL_SUBJECT_SERVO_SETPOINT_3 3013U  /* reg.udral.physics.dynamics.rotation.Planar.0.1 */
#define CYPHAL_SUBJECT_PUMP_SETPOINT    3020U  /* uavcan.primitive.scalar.Bit.1.0               */

/* ---------------------------------------------------------------------------
 * Node lifecycle
 * --------------------------------------------------------------------------- */

/** Initialize Cyphal node (canard, RX queue, subscriptions, TX queue). Call once before scheduler. */
bool cyphal_node_init(void);

/** Start FDCAN RX notifications and the CAN controller. Call after MX_FDCAN1_Init. */
void cyphal_node_start_fdcan(void);

/** Process one cycle: drain RX queue, flush TX queue, apply actuator commands. */
void cyphal_node_step(void);

/** FreeRTOS task entry that calls cyphal_node_step() in a loop. */
void CyphalControlTask(void* pvParameters);

/** Number of RX frames dropped because the queue was full. */
uint32_t cyphal_node_get_frames_dropped(void);

/* ---------------------------------------------------------------------------
 * Typed publish API
 *
 * Fill in the generated struct and pass it here.  Serialization and
 * TX-queue insertion are handled internally; frames are flushed to FDCAN
 * from the task loop.
 * --------------------------------------------------------------------------- */

/**
 * Publish a reg.udral.physics.dynamics.rotation.Planar.0.1 message.
 * Use for servo feedback, state estimation output, or any Planar dynamics topic.
 */
bool cyphal_node_publish_planar(CanardPortID subject_id,
                                const reg_udral_physics_dynamics_rotation_Planar_0_1* msg);

/**
 * Publish a uavcan.primitive.scalar.Bit.1.0 message.
 * Use for boolean status topics (pump state, enable flags, …).
 */
bool cyphal_node_publish_bit(CanardPortID subject_id,
                             const uavcan_primitive_scalar_Bit_1_0* msg);

/**
 * Publish a reg.udral.service.common.Readiness.0.1 message.
 * Use to report this node's readiness/heartbeat state.
 */
bool cyphal_node_publish_readiness(CanardPortID subject_id,
                                   const reg_udral_service_common_Readiness_0_1* msg);

#ifdef __cplusplus
}
#endif

#endif /* CYPHAL_NODE_H */
