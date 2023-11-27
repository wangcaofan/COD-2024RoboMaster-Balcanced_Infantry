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

/**
 * @note PI*0.51*100/(360*1), cm/s
 */
#define WHEEL_DPS_TO_VOLOCITY   0.445058959258554f

/**
 * @note PI*0.085*100/360, cm
 */
#define MOMENTUM_ANGLE_TO_POSITION    0.07417649321f

/**
 * @note PI*0.085*100/(60*1), cm/s 
 */
#define MOMENTUM_RPM_TO_VOLOCITY    0.445058959258554f

/**
 * @note 2*PI/(60*1), rad/s 
 */
#define DJI_GM6020_RPM_TO_RADIANS   0.1047197551f

/**
 * @note 360/(8192*1), digree/s 
 */
#define DJI_GM6020_ENCODER_TO_DIGREES  0.0439453125f

/**
 * @brief typedef enum that contains the mode of chassis
*/
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

/**
 * @brief typedef structure that contains the information of chassis control
*/
typedef struct 
{
  Chassis_Mode_e mode;
	
	uint16_t mode_switch_cnt;
	
	uint16_t balance_switch_cnt;
	
	bool IF_YAW_ANGLE_OFFSET;

  int8_t Chassis_Direction;

  bool IF_Balance_Init;
  float Tp;

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
    float Chassis_Position;
    float Chassis_Velocity;
		float Theta;
    float Last_Theta;
    float Theta_dot;
  }Measure;

  struct
  {
    int16_t Chassis_Speed;
    int16_t Chassis_Position;
  }Limit;
  
  float LQR_K[2][6];
  float LQR_X[6];
  float lQR_Output[2][6];
	

  int16_t SendValue[2][2];
}Control_Info_Typedef;

extern Control_Info_Typedef Control_Info;

#endif //CONTROL_TASK_H
