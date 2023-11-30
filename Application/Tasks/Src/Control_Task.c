/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
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
#include "Control_Task.h"
#include "INS_Task.h"
#include "remote_control.h"
#include "ramp.h"
#include "pid.h"
#include "motor.h"
#include "arm_math.h"
#include "referee_info.h"
#include "VMC_Task.h"

Control_Info_Typedef Control_Info={
    
    .Target.Phi = 0.0f,    // rad
    .mode = CHASSIS_WEAK,
    .Limit.Chassis_Speed = 300,     // cm
    .Limit.Chassis_Position = 100,      // cm
    .LQR_K=
    {
        [0]={0,0,0,0,0,0},  
        [1]={0,0,0,0,0,0},
    },
};

PID_Info_TypeDef Momentum_Init_Pid[2];
PID_Info_TypeDef Wheel_Offset_PID[2];


static void Control_Mode_Update(Control_Info_Typedef *Control_Info);
static void Control_Measure_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info);
static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info);

static void Control_LQR_K_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info);
/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
//int16_t tick;
void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  TickType_t systick = 0;
 /* Infinite loop */
	for(;;)
  {
		//tick = xTaskGetTickCount();
    systick = osKernelSysTick();
		Control_LQR_K_Update(&Control_Info,&Left_VMC_Info);
		Control_Info.Target.Chassis_Position += remote_ctrl.rc.ch[3]*0.0001; 
    Control_Measure_Update(&Control_Info,&Left_VMC_Info);
	  Control_LQR_X_Update(&Control_Info,&Left_VMC_Info);

    osDelayUntil(&systick,1);
  }
  /* USER CODE END Control_Task */
}
static void Control_LQR_K_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info){
  float L0 = VMC_Info->L0;
	Control_Info->LQR_K[0][0]=  -151.8449f*pow(L0,3) + 205.6106f*pow(L0,2) -112.8740f*L0  +1.8701f;
  Control_Info->LQR_K[0][1]=  -2.4723f*pow(L0,3)   +5.3435f*pow(L0,2)    -9.1435f*L0    +0.4514f;
  Control_Info->LQR_K[0][2]=  -22.0389f*pow(L0,3)  +26.1342f*pow(L0,2)   -10.8380f*L0   +0.2090f;
  Control_Info->LQR_K[0][3]=  -31.3419f*pow(L0,3)  +37.4128f*pow(L0,2)   -16.0375f*L0   +0.2787f;
  Control_Info->LQR_K[0][4]=  -13.5837f*pow(L0,3)  +30.7990f*pow(L0,2)  -24.6028f*L0    +8.7899f;
	Control_Info->LQR_K[0][5]=   1.1357f*pow(L0,3)   +0.4961f*pow(L0,2)    -1.7253f*L0     +1.0583f;
	Control_Info->LQR_K[1][0]=   583.8820f*pow(L0,3)  -510.3673f*pow(L0,2) +121.3082f *L0  +9.8385f; 
  Control_Info->LQR_K[1][1]=  53.0943f*pow(L0,3)   -53.3170f*pow(L0,2)  +17.0853f *L0    + 0.0306f;    
  Control_Info->LQR_K[1][2]=  -0.9070f*pow(L0,3)   +18.1948f*pow(L0,2)   -17.6611f *L0   +5.7359f ;  
  Control_Info->LQR_K[1][3]=  2.1872f *pow(L0,3)   +22.0835f*pow(L0,2)   -23.7428f  *L0   +8.0919f;
  Control_Info->LQR_K[1][4]=  501.4394f*pow(L0,3)  -529.0736f*pow(L0,2)  +198.8558f *L0   -11.2287f; 
	Control_Info->LQR_K[1][5]=    54.4287f*pow(L0,3) -59.6999f*pow(L0,2)  + 23.7611f *L0    -1.9947f;  
}
static void Control_Measure_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info){
	  Control_Info->Measure.Phi = INS_Info.angle[2];
    Control_Info->Measure.Phi_dot  = INS_Info.gyro[2];
	  Control_Info->Measure.Chassis_Velocity = (float)(-RMD_Motor[Left_Wheel].Data.Velocity + RMD_Motor[Right_Wheel].Data.Velocity)/2.f;
    Control_Info->Measure.Chassis_Position += Control_Info->Measure.Chassis_Velocity*0.001f;
	  Control_Info->Measure.Theta = (VMC_Info->Phi0- 1.5707965f) + Control_Info->Measure.Phi;
    Control_Info->Measure.Theta_dot =  (Control_Info->Measure.Theta - Control_Info->Measure.Last_Theta)/0.001f;
	  Control_Info->Measure.Last_Theta = Control_Info->Measure.Theta;
 }
static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info){
  Control_Info->LQR_X[0] = Control_Info->Measure.Theta;
  Control_Info->LQR_X[1] = Control_Info->Measure.Theta_dot;
  Control_Info->LQR_X[2] = Control_Info->Target.Chassis_Position - Control_Info->Measure.Chassis_Position;
  Control_Info->LQR_X[3] = Control_Info->Measure.Chassis_Velocity;
  Control_Info->LQR_X[4] = Control_Info->Measure.Phi;
  Control_Info->LQR_X[5] = Control_Info->Measure.Phi_dot;
}
