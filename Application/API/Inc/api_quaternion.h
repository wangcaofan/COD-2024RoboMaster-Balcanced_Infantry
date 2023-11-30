/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_quaternion.h
  * @brief          : quaternion fusion api
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef API_QUATERNION_H
#define API_QUATERNION_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "kalman.h"


/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the Information of Quaternion.
 */
typedef struct 
{
  bool init; /*!< Initialized flag */

  float quat[4];       /*!< the data of quaternion */
  float deviate[3];    /*!< the deviate of Gyro */

  float Q1,Q2,R;       /*!< the data of process/measurement noise covariance matrix */
  float *QuaternionEKF_A_Data;  /*!< pointer to the data of state transition matrix */
  float *QuaternionEKF_P_Data;  /*!< pointer to the data of posteriori covariance matrix */
  KalmanFilter_Info_TypeDef QuaternionEKF;  /*!< Extended Kalman Filter */

  float accel[3];      /*!< the data of accel measure */
  float gyro[3];       /*!< the data of gyro measure */
  float accelInvNorm;       /*!< the inverse of accel norm */
  float gyroInvNorm;        /*!< the inverse of gyro norm */
  float halfgyrodt[3];      /*!< half gyro dt */
  float EulerAngle[3];      /*!< Euler angles in radians: */
}Quaternion_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the Quaternion EKF according to the specified parameters in the Quaternion_Info_Typedef.
  */
extern void QuaternionEKF_Init(Quaternion_Info_Typedef *quat,float process_noise1,float process_noise2,float measure_noise,float *QuaternionEKF_A_Data,float *QuaternionEKF_P_Data);
/**
  * @brief  Update the Extended Kalman Filter
  */
extern void QuaternionEKF_Update(Quaternion_Info_Typedef *quat,float gyro[3],float accel[3],float dt);

#endif //API_QUATERNION_H


