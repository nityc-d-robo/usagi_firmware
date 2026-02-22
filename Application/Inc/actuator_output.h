/**
 * @file actuator_output.h
 * @brief PWM output: 4 servos (TIM2 CH1-4), 1 pump (TIM1 CH1 + direction GPIO).
 */

#ifndef ACTUATOR_OUTPUT_H
#define ACTUATOR_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Servo setpoints: rad or rad/s; applied as -1..1 to pulse width. Pump: on/off. Readiness: 0/2/3. */
void actuator_output_apply(const float servo_setpoints[4], bool pump_on, uint8_t readiness);

void actuator_output_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ACTUATOR_OUTPUT_H */
