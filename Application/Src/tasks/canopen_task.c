#include "canopen_task.h"
#include "fdcan.h"
#include "tim.h"
#include "CO_app_STM32.h"
#include "CANopen.h"
#include "OD.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

void canopen_task(void *argument)
{
  /* USER CODE BEGIN canopen_task */
  CANopenNodeSTM32 canOpenNodeSTM32;
  canOpenNodeSTM32.CANHandle = &hfdcan1;
  canOpenNodeSTM32.HWInitFunction = MX_FDCAN1_Init;
  canOpenNodeSTM32.timerHandle = &htim17;
  canOpenNodeSTM32.desiredNodeID = 21;
  canOpenNodeSTM32.baudrate = 1000;
  canopen_app_init(&canOpenNodeSTM32);

  uint32_t hb_log_tick = 0;

  /* Infinite loop */
  for(;;)
  {
    canopen_app_process();

    uint32_t now = HAL_GetTick();
    if (now - hb_log_tick >= 1000)
    {
      hb_log_tick = now;

      printf("[SERVO] CH1=%u CH2=%u CH3=%u CH4=%u [MOTOR] spd=%d\r\n",
             OD_RAM.x2000_servoPositions[0],
             OD_RAM.x2000_servoPositions[1],
             OD_RAM.x2000_servoPositions[2],
             OD_RAM.x2000_servoPositions[3],
             OD_RAM.x2001_motorSpeed);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END canopen_task */
}
