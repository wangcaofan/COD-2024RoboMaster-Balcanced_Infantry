/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Control_Task.h
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

/* Exported defines -----------------------------------------------------------*/
#define SHOOT_SPEED_15M_S    4600
#define SHOOT_SPEED_18M_S    5100
#define SHOOT_SPEED_30M_S    7450

#define TRIGGER_FREQ_2_HZ     750
#define TRIGGER_FREQ_3_HZ     1000
#define TRIGGER_FREQ_4_HZ     1250
#define TRIGGER_FREQ_5_HZ     1500
#define TRIGGER_FREQ_6_HZ     1750
#define TRIGGER_FREQ_7_HZ     2000
#define TRIGGER_FREQ_8_HZ     2400
#define TRIGGER_FREQ_9_HZ     2700
#define TRIGGER_FREQ_10_HZ    3000

#define SignBit(x)  ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the Shooter Mode.
 */
typedef enum{
  SHOOTER_OFF,
  SHOOTER_SINGLE,
  SHOOTER_BURSTS,
  SHOOTER_VISION,
  SHOOTER_MODE_NUM,
}Shooter_Mode_e;

/**
 * @brief typedef enum that contains the Gimbaol Mode.
 */
typedef enum{
	GIMBAL_OFF,
  GIMBAL_Calibration,
  GIMBAL_IMU,
	GIMBAL_VISION,
  GIMBAL_MODE_NUM,
}Gimbal_Mode_e;

/**
 * @brief typedef structure that contains the information for the gimbal control.
 */
typedef struct
{
  Shooter_Mode_e shoot_mode;
  Gimbal_Mode_e gimbal_mode;
  uint8_t trigger_Buf;
  bool FIRE_SINGLE_ENABLE;
	
	int8_t stuck_flag;

	struct{
		float pit_angle;
		float yaw_angle;
		float pit_gyro;
		float yaw_gyro;
    int16_t shootSpeed;
    float triggerSpeed;
    float triggerAngle;
	}Target;
	
	struct{
		float *pit_angle;
		float *yaw_angle;
		float *pit_gyro;
		float *yaw_gyro;
    int16_t *shootLeftSpeed;
    int16_t *shootRightSpeed;
    int16_t *triggerSpeed;
    float *triggerAngle;
	}Measure;
	
	struct{
		float min;
		float max;
	}Limit_pitch;
		
	float Fire_Speed_Offset;
	float last_Firespeed;
	
	int16_t SendValue[5];

}Control_Info_Typedef;

/* Exported variables ----------------------------------------------------------*/
extern Control_Info_Typedef Control_Info;

#endif // CONTROL_TASK_H
