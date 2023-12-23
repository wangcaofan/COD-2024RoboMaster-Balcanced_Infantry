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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"



typedef enum 
{
  CHASSIS_WEAK,
  CHASSIS_STOP,
  CHASSIS_BALANCE,
  CHASSIS_FRONT,
  CHASSIS_SIDE,
  CHASSIS_SPIN,
  CHASSIS_SLIP,
  CHASSIS_MODE_NUM,
}Chassis_Mode_e;
typedef enum{
  Left_Control,
  Right_Control,
  Control_USAGE_NUM,
}Control_USAGE_e;
/**
 * @brief typedef structure that contains the information of chassis control
*/
typedef struct 
{
  Chassis_Mode_e mode;
	Control_USAGE_e Control_USAGE;
	uint16_t mode_switch_cnt;
	
	uint16_t balance_switch_cnt;
  struct
  {
		float Phi;
		float Phi_dot;
    float Chassis_Position;
    float Chassis_Velocity;
    float Theta;
    float Theta_dot;
  }Target;

  struct
  {
		float Phi;
		float Phi_dot;
		float Last_Phi;
    float Chassis_Position;
    float Chassis_Velocity;
		float Theta;
    float Last_Theta;
    float Theta_dot;
  }Measure;
  
	struct{
	 	float T;
		float Tp;
		float Leg_Coordinate_Tp;
		float Stand_T;
	  float Turn_T;
  }Moment;
  
	struct
  {
    int16_t Chassis_Speed;
    int16_t Chassis_Position;
  }Limit;
  
  float LQR_K[2][6];
  float LQR_X[6];
  float LQR_Output[2][6];

  int16_t rc_tick;

	struct
  {
    float T1;
    float T2;
  }SendValue;
}Control_Info_Typedef;



typedef struct 
{
	Control_USAGE_e VMC_USAGE;
  
	float L1,L2,L3,L4,L5,L0,Last_L0,L0_dot,Target_L0;
	float *Phi1,*Phi4;
	float  Phi2,Phi3,Phi0,Last_Phi0;
	float *Phi1_dot,*Phi4_dot,Phi0_dot;
	float X_B,Y_B,X_D,Y_D,X_C,Y_C,Y_C_dot,X_C_dot;
	float F,T1,T2,Tp;
	float Theta;

}VMC_Info_Typedef;

extern VMC_Info_Typedef Left_VMC_Info;
extern VMC_Info_Typedef Right_VMC_Info;
extern Control_Info_Typedef Left_Control_Info;
extern Control_Info_Typedef Right_Control_Info;
#endif //CONTROL_TASK_H
