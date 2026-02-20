#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "tim.h"
#include "servo_control.h"

void ServoControl(void* pvParameters)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
