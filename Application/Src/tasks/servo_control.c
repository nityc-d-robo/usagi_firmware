#include "FreeRTOS.h"
#include "task.h"

#include "CANopen.h"
#include "OD.h"
#include "tim.h"
#include "servo_control.h"

void ServoControl(void* pvParameters)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    for (;;) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, OD_RAM.x2000_servoPositions[0]);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, OD_RAM.x2000_servoPositions[1]);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, OD_RAM.x2000_servoPositions[2]);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, OD_RAM.x2000_servoPositions[3]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
