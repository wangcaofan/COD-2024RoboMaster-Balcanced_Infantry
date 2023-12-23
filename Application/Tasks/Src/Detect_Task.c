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
#include "VMC_Task.h"
#include "INS_Task.h"
#include "motor.h"
/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
//void usart_printf(const char *fmt,...)
//{
//   
//    static va_list ap;
//    static uint16_t len;
//    static uint8_t tx_buf[256] = {0};
//    va_start(ap, fmt);
//    len = vsnprintf((char *)tx_buf,sizeof(tx_buf)+1 ,(char*)fmt, ap);

//    va_end(ap);

//    HAL_UART_Transmit_DMA(&huart1,(uint8_t*)tx_buf, len);
//    
//}
//#define printf(title, fmt, args...) usart_printf("{"#title"}"fmt"\n", ##args)
void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
 TickType_t systick = 0;

  /* Infinite loop */
  for(;;)
  {
  systick = osKernelSysTick();

//   //Start_UI_task();
////    printf(phi0,"%f,%f",Damiao_Motor[0].Data.Predict_Velocity,Damiao_Motor[0].Data.Velocity);
		// printf(phi0,"%f",Left_VMC_Info.Phi0_dot);
  //osDelayUntil(&systick,1);
  }
  /* USER CODE END Detect_Task */
}

