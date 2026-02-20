#include "canopen_task.h"
#include "fdcan.h"
#include "tim.h"
#include "CO_app_STM32.h"
#include "CANopen.h"
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

      CO_t *co = canOpenNodeSTM32.canOpenStack;
      CO_NMT_internalState_t state = CO_NMT_getInternalState(co->NMT);

      /* FDCAN Protocol Status Register */
      uint32_t psr = hfdcan1.Instance->PSR;
      /* FDCAN Error Counter Register */
      uint32_t ecr = hfdcan1.Instance->ECR;
      uint8_t  tec = (ecr & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos;
      uint8_t  rec = (ecr & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos;

      printf("[CAN] NMT=%d TEC=%u REC=%u BusOff=%lu ErrPassive=%lu ErrWarn=%lu\r\n",
             (int)state,
             tec, rec,
             (psr & FDCAN_PSR_BO) ? 1UL : 0UL,
             (psr & FDCAN_PSR_EP) ? 1UL : 0UL,
             (psr & FDCAN_PSR_EW) ? 1UL : 0UL);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END canopen_task */
}
