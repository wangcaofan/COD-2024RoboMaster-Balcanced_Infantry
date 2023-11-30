/**
  ******************************************************************************
  * @file           : Decision_Task.c
  * @brief          : Decision task
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
#include "Decision_Task.h"
#include "Control_Task.h"



/* USER CODE BEGIN Header_Decision_Task */
/**
* @brief Function implementing the StartDecisionTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Decision_Task */
void Decision_Task(void const * argument)
{
  /* USER CODE BEGIN Decision_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Decision_Task */
}


