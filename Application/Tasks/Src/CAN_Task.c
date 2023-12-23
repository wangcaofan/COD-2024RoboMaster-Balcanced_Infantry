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
#include "INS_Task.h"
#include "motor.h"
#include "bsp_can.h"
#include "referee_info.h"
#include "VMC_Task.h"
#include "remote_control.h"


static void Damiao_Motor_Enable(uint8_t ID);
static void Damiao_Motor_DisEnable(uint8_t ID);
static void Damiao_Motor_CAN_Send(uint8_t ID,float Postion, float Velocity, float KP, float KD, float Torque);
static int float_to_uint(float x, float x_min, float x_max, int bits);
//static float Damiao_Motor_Ang
/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
int16_t SendValue,Torgue1,Torgue2=0;
float Speed = 0;
void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
  TickType_t systick = 0;
	 osDelay(600);
	Damiao_Motor_Enable(0x01);
    osDelay(10);
  Damiao_Motor_Enable(0x02);
    osDelay(10);
	Damiao_Motor_Enable(0x03);
    osDelay(10);
  Damiao_Motor_Enable(0x04);
    osDelay(10);
  /* Infinite loop */
  for(;;)
  {
		 systick = osKernelSysTick();
//		if(remote_ctrl.rc.s[1]==1){
//		//Torgue=Left_Control_Info.SendValue*332.58f;
//		}else	Torgue=  0;

//		RMD_L9025_Right_TxFrame.Data[0] = 0xA1;
//    RMD_L9025_Right_TxFrame.Data[4] = (uint8_t)(Torgue);
//    RMD_L9025_Right_TxFrame.Data[5] = (uint8_t)(Torgue>>8);
//	  USER_CAN_TxMessage(&RMD_L9025_Right_TxFrame);

//	  Damiao_Motor_CAN_Send(0x01,0,0,0,0,Left_VMC_Info.T1);
//	  
//    Damiao_Motor_CAN_Send(0x02,0,0,0,0,Left_VMC_Info.T2);
		if(remote_ctrl.rc.s[1]==1){


	  Damiao_Motor_CAN_Send(0x01,0,0,0,0,Left_VMC_Info.T1);
    Damiao_Motor_CAN_Send(0x02,0,0,0,0,Left_VMC_Info.T2);
		osDelay(1);
		Damiao_Motor_CAN_Send(0x03,0,0,0,0,Right_VMC_Info.T2);
    Damiao_Motor_CAN_Send(0x04,0,0,0,0,Right_VMC_Info.T1);
				Torgue1 = Right_Control_Info.Moment.T*332.58f;
		    Torgue2 = Left_Control_Info.Moment.T*332.58f;
		SendValue = (-Torgue1+Torgue2)/2;
		RMD_L9025_Left_TxFrame.Data[0] = 0xA1;
		RMD_L9025_Left_TxFrame.Data[4] = (uint8_t)(Torgue1);
    RMD_L9025_Left_TxFrame.Data[5] = (uint8_t)(Torgue1>>8);
		USER_CAN_TxMessage(&RMD_L9025_Left_TxFrame);
    RMD_L9025_Right_TxFrame.Data[0] = 0xA1;
    RMD_L9025_Right_TxFrame.Data[4] = (uint8_t)(Torgue2);
    RMD_L9025_Right_TxFrame.Data[5] = (uint8_t)(Torgue2>>8);
	  USER_CAN_TxMessage(&RMD_L9025_Right_TxFrame);

		}else {

    Damiao_Motor_CAN_Send(0x01,0,0,0,0,0);
	  Damiao_Motor_CAN_Send(0x02,0,0,0,0,0);
	  osDelay(1);
		Damiao_Motor_CAN_Send(0x03,0,0,0,0,0);
    Damiao_Motor_CAN_Send(0x04,0,0,0,0,0);
		RMD_L9025_Left_TxFrame.Data[0] = 0xA1;
		RMD_L9025_Left_TxFrame.Data[4] = (uint8_t)(0);
    RMD_L9025_Left_TxFrame.Data[5] = (uint8_t)(0>>8);
		USER_CAN_TxMessage(&RMD_L9025_Left_TxFrame);
		RMD_L9025_Right_TxFrame.Data[0] = 0xA1;
    RMD_L9025_Right_TxFrame.Data[4] = (uint8_t)(0);
    RMD_L9025_Right_TxFrame.Data[5] = (uint8_t)(0>>8);
	  USER_CAN_TxMessage(&RMD_L9025_Right_TxFrame);
		
		}
    osDelay(1);

  }
  /* USER CODE END CAN_Task */
}
static void Damiao_Motor_Enable(uint8_t ID){
	 uint8_t Motor_ID =ID-1;
   JointTxFrame[Motor_ID].Data[0] = 0xFF;
   JointTxFrame[Motor_ID].Data[1] = 0xFF;
   JointTxFrame[Motor_ID].Data[2] = 0xFF;
   JointTxFrame[Motor_ID].Data[3] = 0xFF;
   JointTxFrame[Motor_ID].Data[4] = 0xFF;
   JointTxFrame[Motor_ID].Data[5] = 0xFF;
   JointTxFrame[Motor_ID].Data[6] = 0xFF;
	 JointTxFrame[Motor_ID].Data[7] = 0xFC;
	 USER_CAN_TxMessage(&JointTxFrame[Motor_ID]);
}
static void Damiao_Motor_CAN_Send(uint8_t ID,float Postion, float Velocity, float KP, float KD, float Torque){
   static uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
	 uint8_t Motor_ID = ID-1;
   Postion_Tmp  =  float_to_uint(Postion,-3.141593f,3.141593,16) ;
   Velocity_Tmp =  float_to_uint(Velocity,-50,50,12);
	 KP_Tmp = float_to_uint(KP,0,500,12);
	 KD_Tmp = float_to_uint(KD,0,5,12);
   Torque_Tmp = float_to_uint(Torque,-45,45,12);
	 JointTxFrame[Motor_ID].Data[0] = (uint8_t)(Postion_Tmp>>8);
	 JointTxFrame[Motor_ID].Data[1] = (uint8_t)(Postion_Tmp);
	 JointTxFrame[Motor_ID].Data[2] = (uint8_t)(Velocity_Tmp>>4);
	 JointTxFrame[Motor_ID].Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
	 JointTxFrame[Motor_ID].Data[4] = (uint8_t)(KP_Tmp);
	 JointTxFrame[Motor_ID].Data[5] = (uint8_t)(KD_Tmp>>4);
	 JointTxFrame[Motor_ID].Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
	 JointTxFrame[Motor_ID].Data[7] = (uint8_t)(Torque_Tmp);
   USER_CAN_TxMessage(&JointTxFrame[Motor_ID]);
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
