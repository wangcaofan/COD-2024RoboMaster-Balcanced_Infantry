/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
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
#include "CAN_Task.h"
#include "Control_Task.h"
#include "Vision_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "remote_control.h"

uint8_t MotorCAN_FramInfo[4][8];

static void Remote_Control_TxCANFrame(Remote_Info_Typedef *remote_ctrl,uint8_t data[8]);

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
  osDelay(100);
  
  /* Infinite loop */
  for(;;)
  {
    MotorCAN_FramInfo[0][DJI_Motor[Pitch].CANFrame.FrameIndex]   = (uint8_t)(Control_Info.SendValue[Pitch] >> 8);
    MotorCAN_FramInfo[0][DJI_Motor[Pitch].CANFrame.FrameIndex+1] = (uint8_t)(Control_Info.SendValue[Pitch]);
    USER_CAN_TxMessage(CAN1,DJI_Motor[Pitch].CANFrame.TxStdId,MotorCAN_FramInfo[0],DJI_Motor[Pitch].CANFrame.FrameIndex+2);

    MotorCAN_FramInfo[1][DJI_Motor[Yaw].CANFrame.FrameIndex]   = (uint8_t)(Control_Info.SendValue[Yaw] >> 8);
    MotorCAN_FramInfo[1][DJI_Motor[Yaw].CANFrame.FrameIndex+1] = (uint8_t)(Control_Info.SendValue[Yaw]);
    USER_CAN_TxMessage(CAN2,DJI_Motor[Yaw].CANFrame.TxStdId,MotorCAN_FramInfo[1],DJI_Motor[Yaw].CANFrame.FrameIndex+2);

    MotorCAN_FramInfo[2][DJI_Motor[SHOOTL].CANFrame.FrameIndex]   = (uint8_t)(Control_Info.SendValue[SHOOTL] >> 8);
    MotorCAN_FramInfo[2][DJI_Motor[SHOOTL].CANFrame.FrameIndex+1] = (uint8_t)(Control_Info.SendValue[SHOOTL]);
    MotorCAN_FramInfo[2][DJI_Motor[SHOOTR].CANFrame.FrameIndex]   = (uint8_t)(Control_Info.SendValue[SHOOTR] >> 8);
    MotorCAN_FramInfo[2][DJI_Motor[SHOOTR].CANFrame.FrameIndex+1] = (uint8_t)(Control_Info.SendValue[SHOOTR]);
    MotorCAN_FramInfo[2][DJI_Motor[TRIGGER].CANFrame.FrameIndex]   = (uint8_t)(Control_Info.SendValue[TRIGGER] >> 8);
    MotorCAN_FramInfo[2][DJI_Motor[TRIGGER].CANFrame.FrameIndex+1] = (uint8_t)(Control_Info.SendValue[TRIGGER]);
    USER_CAN_TxMessage(CAN1,DJI_Motor[TRIGGER].CANFrame.TxStdId,MotorCAN_FramInfo[2],DJI_Motor[TRIGGER].CANFrame.FrameIndex+2);

    Remote_Control_TxCANFrame(&remote_ctrl,MotorCAN_FramInfo[3]);

    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

bool IF_Shoot_Accept = false;

static void Remote_Control_TxCANFrame(Remote_Info_Typedef *remote_ctrl,uint8_t data[8])
{
	IF_Shoot_Accept = ((abs(DJI_Motor[SHOOTL].Data.velocity) + abs(DJI_Motor[SHOOTL].Data.velocity))/2.f > 2000);
	
	data[0] = (uint8_t)(remote_ctrl->rc.s[0]) << 6 | \
						(uint8_t)(remote_ctrl->rc.ch[4]== 660) << 5 | \
						(uint8_t)(remote_ctrl->rc.ch[4]==-660) << 4;
	
	data[1] = (uint8_t)(IF_Shoot_Accept) << 4 | \
						(uint8_t)(Control_Info.shoot_mode) << 2 | \
						(uint8_t)(Vision_Info.IF_Aiming_Enable && Mouse_Pressed_Right()) << 1 | \
						(uint8_t)(Key_R());

	data[2] = (uint8_t)(remote_ctrl->rc.ch[3] >> 8);
	data[3] = (uint8_t)(remote_ctrl->rc.ch[3]);
	data[4] = (uint8_t)(remote_ctrl->rc.ch[2] >> 8);
	data[5] = (uint8_t)(remote_ctrl->rc.ch[2]);
	data[6] = (uint8_t)(remote_ctrl->key.v >> 8);
	data[7] = (uint8_t)(remote_ctrl->key.v);

	USER_CAN_TxMessage(CAN2,0x302,data,8);
}
