#include "FreeRTOS.h"
#include "task.h"

#include "CANopen.h"
#include "OD.h"
#include "tim.h"
#include "main.h"
#include "motor_control.h"

void MotorControl(void* pvParameters)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    for (;;) {
        int16_t spd = OD_RAM.x2001_motorSpeed;
        uint32_t duty = (spd < 0) ? (uint32_t)(-spd) : (uint32_t)spd;
        if (duty > 999) duty = 999;
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1,
            (spd >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
