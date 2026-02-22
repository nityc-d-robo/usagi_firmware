/**
 * @file actuator_command.c
 * @brief Planar/Bit/Readiness decode, timeout, and apply to output.
 */

#include "actuator_command.h"
#include "actuator_output.h"
#include "cyphal_node.h"
#include "dsdl_runtime.h"
#include <dsdl.h>
#include "canard.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>

#define CONTROL_TIMEOUT_MS 1000U
#define READINESS_ENGAGED  3U


/* Command state: 4 servos (Planar: use angular_position for PWM), 1 pump (bool), readiness. */
static float s_servo[4];
static bool s_pump_on;
static uint8_t s_readiness;
static TickType_t s_last_cmd_tick;
static uint32_t s_decode_errors;
static uint32_t s_timeout_count;
static bool s_in_timeout;

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

static void decode_planar(uint8_t idx, const void* payload, size_t size)
{
    if (idx >= 4) {
        return;
    }
    const struct dsdl_type_composite_t* type = dsdl_runtime_type_planar();
    if (type == NULL) {
        return; /* dsdl not inited yet */
    }
    /* Planar.0.1: kinematics (angular_position, angular_velocity, angular_acceleration), torque. */
    float angular_position = 0.0f;
    float angular_velocity = 0.0f;
    float angular_acceleration = 0.0f;
    float torque = 0.0f;
    void* kin_vals[3] = { &angular_position, &angular_velocity, &angular_acceleration };
    struct dsdl_value_struct_t kin_st = { .values = kin_vals };
    void* planar_vals[2] = { &kin_st, &torque };
    struct dsdl_value_struct_t planar_st = { .values = planar_vals };
    dsdl_error_t err = dsdl_error_none;
    if (dsdl_deserialize(type, &planar_st, size, payload, &err) != (size_t)-1) {
        /* Use first finite: position, then velocity, then 0. Clamp to -1..1 for PWM scaling. */
        float sp = 0.0f;
        if (isfinite(angular_position)) {
            sp = angular_position;
        } else if (isfinite(angular_velocity)) {
            sp = angular_velocity;
        }
        if (sp > 1.0f) sp = 1.0f;
        if (sp < -1.0f) sp = -1.0f;
        s_servo[idx] = sp;
    } else {
        s_decode_errors++;
    }
}

static void decode_bit(const void* payload, size_t size)
{
    if (size < 1) {
        return;
    }
    const struct dsdl_type_composite_t* type = dsdl_runtime_type_bit();
    if (type == NULL) {
        return;
    }
    bool val = false;
    void* values[1] = { &val };
    dsdl_value_struct_t st = { .values = values };
    dsdl_error_t err = dsdl_error_none;
    if (dsdl_deserialize(type, &st, size, payload, &err) != (size_t)-1) {
        s_pump_on = val;
    } else {
        s_decode_errors++;
    }
}

static void decode_readiness(const void* payload, size_t size)
{
    if (size < 1) {
        return;
    }
    const struct dsdl_type_composite_t* type = dsdl_runtime_type_readiness();
    if (type == NULL) {
        return;
    }
    uint8_t val = 0;
    void* values[1] = { &val };
    dsdl_value_struct_t st = { .values = values };
    dsdl_error_t err = dsdl_error_none;
    if (dsdl_deserialize(type, &st, size, payload, &err) != (size_t)-1) {
        s_readiness = val & 3u;
    } else {
        s_decode_errors++;
    }
}

void actuator_command_on_transfer(const struct CanardRxTransfer* transfer,
                                  const struct CanardRxSubscription* subscription)
{
    if (transfer == NULL || subscription == NULL) {
        return;
    }
    s_last_cmd_tick = xTaskGetTickCount();
    const CanardPortID port_id = subscription->port_id;
    size_t size = transfer->payload.size;
    const uint8_t* data = transfer->payload.data;
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
    TickType_t now = xTaskGetTickCount();
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
