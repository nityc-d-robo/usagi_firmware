#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "led_blink_node.h"

void LEDBlinkTask(void* pvParameters)
{
  while (1)
  {
    BSP_LED_Toggle(LED_GREEN);
    char* uart_mes = "LED Blink!\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)uart_mes, strlen(uart_mes), 1000);
    vTaskDelay(pdMS_TO_TICKS(1000));   
  }
}
