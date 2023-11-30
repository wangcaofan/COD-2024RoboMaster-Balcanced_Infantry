/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId StartINSTaskHandle;
osThreadId StartDetectTaskHandle;
osThreadId StartControlTasHandle;
osThreadId StartCANTaskHandle;
osThreadId StartVMC_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_Task(void const * argument);
void Detect_Task(void const * argument);
void Control_Task(void const * argument);
void CAN_Task(void const * argument);
void VMC_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StartINSTask */
  osThreadDef(StartINSTask, INS_Task, osPriorityRealtime, 0, 256);
  StartINSTaskHandle = osThreadCreate(osThread(StartINSTask), NULL);

  /* definition and creation of StartDetectTask */
  osThreadDef(StartDetectTask, Detect_Task, osPriorityRealtime, 0, 256);
  StartDetectTaskHandle = osThreadCreate(osThread(StartDetectTask), NULL);

  /* definition and creation of StartControlTas */
  osThreadDef(StartControlTas, Control_Task, osPriorityRealtime, 0, 256);
  StartControlTasHandle = osThreadCreate(osThread(StartControlTas), NULL);

  /* definition and creation of StartCANTask */
  osThreadDef(StartCANTask, CAN_Task, osPriorityRealtime, 0, 256);
  StartCANTaskHandle = osThreadCreate(osThread(StartCANTask), NULL);

  /* definition and creation of StartVMC_Task */
  osThreadDef(StartVMC_Task, VMC_Task, osPriorityRealtime, 0, 256);
  StartVMC_TaskHandle = osThreadCreate(osThread(StartVMC_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
__weak void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

/* USER CODE BEGIN Header_VMC_Task */
/**
* @brief Function implementing the StartVMC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VMC_Task */
__weak void VMC_Task(void const * argument)
{
  /* USER CODE BEGIN VMC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END VMC_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
