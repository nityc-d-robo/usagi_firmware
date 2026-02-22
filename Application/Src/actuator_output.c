/**
 * @file actuator_output.c
 * @brief Apply servo (TIM2) and pump (TIM1 + PF1) from command state.
 */

#include "actuator_output.h"
#include "tim.h"
#include "main.h"
#include <math.h>

#define SERVO_NEUTRAL_US  1500   /* center [us] */
#define SERVO_MIN_US       900   /* lower limit [us] */
#define SERVO_MAX_US      2100   /* upper limit [us] */
#define SERVO_RANGE_US     600   /* half range: (2100 - 1500) = 600; setpoint ±1 → min/max */
#define SERVO_PERIOD_TICKS 19999
#define PUMP_DUTY_FIXED    400
#define PUMP_PERIOD        999

static uint32_t setpoint_to_servo_ticks(float setpoint)
{
    float pulse_us = (float)SERVO_NEUTRAL_US + setpoint * (float)SERVO_RANGE_US;
    if (pulse_us < (float)SERVO_MIN_US) pulse_us = (float)SERVO_MIN_US;
    if (pulse_us > (float)SERVO_MAX_US) pulse_us = (float)SERVO_MAX_US;
    return (uint32_t)(pulse_us * (float)(SERVO_PERIOD_TICKS + 1) / 20000.0f);
}

void actuator_output_init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void actuator_output_apply(const float servo_setpoints[4], bool pump_on, uint8_t readiness)
{
    const bool engaged = (readiness == 3u);
    if (!engaged) {
        const uint32_t neutral = setpoint_to_servo_ticks(0.0f);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, neutral);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, neutral);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, neutral);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, neutral);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
        return;
    }
    if (servo_setpoints != NULL) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, setpoint_to_servo_ticks(servo_setpoints[0]));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, setpoint_to_servo_ticks(servo_setpoints[1]));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, setpoint_to_servo_ticks(servo_setpoints[2]));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, setpoint_to_servo_ticks(servo_setpoints[3]));
    }
    if (pump_on) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PUMP_DUTY_FIXED);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
    }
}
