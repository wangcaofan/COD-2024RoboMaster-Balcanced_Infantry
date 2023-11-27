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
#include "Control_Task.h"
#include "remote_control.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"

/**
  * @note turn on:  800
	*       turn off: 4150
	*/
uint16_t Cover_PWM_Compare = 0;

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
//  TickType_t systick = 0;
	
	osDelay(50);
  /* Infinite loop */
  for(;;)
  {
//    systick = osKernelSysTick();

    /* cover pwm control */
    if(Key_R() == false)
    {
      Cover_PWM_Compare = 4200;
    }
    else
    {
      Cover_PWM_Compare = 1250;
    }
		
		if(remote_ctrl.rc.ch[4] == -660)
		{
      Cover_PWM_Compare = 1250;
		}
		
    Cover_Control(Cover_PWM_Compare);
		
		if(Control_Info.gimbal_mode == GIMBAL_VISION || Control_Info.shoot_mode == SHOOTER_VISION)
		{
				LASER_TURN_OFF();
		}
		else
		{
				LASER_TURN_ON();
		}
		

    /* remote control moniter */
    Remote_Message_Moniter(&remote_ctrl);

    osDelay(5);
  }
  /* USER CODE END Detect_Task */
}

