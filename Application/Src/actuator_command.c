/**
 * @file actuator_command.c
 * @brief Planar/Bit/Readiness decode via nunavut-generated types, timeout, and apply to output.
 */

#include "actuator_command.h"
#include "actuator_output.h"
#include "cyphal_node.h"
#include "canard.h"
#include "FreeRTOS.h"
#include "task.h"
#include <reg/udral/physics/dynamics/rotation/Planar_0_1.h>
#include <reg/udral/service/common/Readiness_0_1.h>
#include <uavcan/primitive/scalar/Bit_1_0.h>
#include <math.h>

#define CONTROL_TIMEOUT_MS 1000U
#define READINESS_ENGAGED  3U

/* Command state: 4 servos (Planar angular_position for PWM), 1 pump (bool), readiness. */
static float    s_servo[4];
static bool     s_pump_on;
static uint8_t  s_readiness;
static TickType_t s_last_cmd_tick;
static uint32_t s_decode_errors;
static uint32_t s_timeout_count;
static bool     s_in_timeout;

static void apply_safe_state(void)
{
    s_readiness = 0;
    s_pump_on   = false;
    for (int i = 0; i < 4; i++) {
        s_servo[i] = 0.0f;
    }
    actuator_output_apply(s_servo, s_pump_on, s_readiness);
}

void actuator_command_init(void)
{
    s_pump_on       = false;
    s_readiness     = 0;
    s_last_cmd_tick = 0;
    s_decode_errors = 0;
    s_timeout_count = 0;
    s_in_timeout    = false;
    for (int i = 0; i < 4; i++) {
        s_servo[i] = 0.0f;
    }
    actuator_output_init();
    apply_safe_state();
}

static void decode_planar(uint8_t idx, const uint8_t* payload, size_t size)
{
    if (idx >= 4) {
        return;
    }
    reg_udral_physics_dynamics_rotation_Planar_0_1 msg;
    reg_udral_physics_dynamics_rotation_Planar_0_1_initialize_(&msg);
    size_t buf_size = size;
    if (reg_udral_physics_dynamics_rotation_Planar_0_1_deserialize_(&msg, payload, &buf_size) < 0) {
        s_decode_errors++;
        return;
    }
    /* Use first finite value: position, then velocity, then leave at 0. */
    float sp = 0.0f;
    const float pos = msg.kinematics.angular_position.radian;
    const float vel = msg.kinematics.angular_velocity.radian_per_second;
    if (isfinite(pos)) {
        sp = pos;
    } else if (isfinite(vel)) {
        sp = vel;
    }
    if (sp > 1.0f)  sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;
    s_servo[idx] = sp;
}

static void decode_bit(const uint8_t* payload, size_t size)
{
    if (size < 1) {
        return;
    }
    uavcan_primitive_scalar_Bit_1_0 msg;
    uavcan_primitive_scalar_Bit_1_0_initialize_(&msg);
    size_t buf_size = size;
    if (uavcan_primitive_scalar_Bit_1_0_deserialize_(&msg, payload, &buf_size) < 0) {
        s_decode_errors++;
        return;
    }
    s_pump_on = msg.value;
}

static void decode_readiness(const uint8_t* payload, size_t size)
{
    if (size < 1) {
        return;
    }
    reg_udral_service_common_Readiness_0_1 msg;
    reg_udral_service_common_Readiness_0_1_initialize_(&msg);
    size_t buf_size = size;
    if (reg_udral_service_common_Readiness_0_1_deserialize_(&msg, payload, &buf_size) < 0) {
        s_decode_errors++;
        return;
    }
    s_readiness = msg.value & 3u;
}

void actuator_command_on_transfer(const struct CanardRxTransfer* transfer,
                                  const struct CanardRxSubscription* subscription)
{
    if (transfer == NULL || subscription == NULL) {
        return;
    }
    s_last_cmd_tick = xTaskGetTickCount();
    const CanardPortID port_id = subscription->port_id;
    const size_t       size    = transfer->payload.size;
    const uint8_t*     data    = transfer->payload.data;
    if (data == NULL) {
        return;
    }

    if (port_id == CYPHAL_SUBJECT_READINESS) {
        decode_readiness(data, size);
    } else if (port_id == CYPHAL_SUBJECT_SERVO_SETPOINT_0) {
        decode_planar(0, data, size);
    } else if (port_id == CYPHAL_SUBJECT_SERVO_SETPOINT_1) {
        decode_planar(1, data, size);
    } else if (port_id == CYPHAL_SUBJECT_SERVO_SETPOINT_2) {
        decode_planar(2, data, size);
    } else if (port_id == CYPHAL_SUBJECT_SERVO_SETPOINT_3) {
        decode_planar(3, data, size);
    } else if (port_id == CYPHAL_SUBJECT_PUMP_SETPOINT) {
        decode_bit(data, size);
    }
}

void actuator_command_apply(void)
{
    const TickType_t now = xTaskGetTickCount();
    if ((now - s_last_cmd_tick) > pdMS_TO_TICKS(CONTROL_TIMEOUT_MS)) {
        if (!s_in_timeout) {
            s_timeout_count++;
            s_in_timeout = true;
        }
        apply_safe_state();
    } else {
        s_in_timeout = false;
        actuator_output_apply(s_servo, s_pump_on, s_readiness);
    }
}

void actuator_command_get_stats(uint32_t* decode_errors, uint32_t* timeout_count)
{
    if (decode_errors != NULL) {
        *decode_errors = s_decode_errors;
    }
    if (timeout_count != NULL) {
        *timeout_count = s_timeout_count;
    }
}
