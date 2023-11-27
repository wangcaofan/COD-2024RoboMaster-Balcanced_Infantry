/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Detect_Task.c
  * @brief          : Detect task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Detect_Task.h"
#include "remote_control.h"
#include "UI.h"

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
// TickType_t systick = 0;

  /* Infinite loop */
  for(;;)
  {
//   systick = osKernelSysTick();

   //Start_UI_task();

   osDelay(10);
  }
  /* USER CODE END Detect_Task */
}

