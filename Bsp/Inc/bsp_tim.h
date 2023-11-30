/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tim.h
  * @brief          : bsp tim functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to config the clock source of the advanced TIM
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_TIM_H
#define BSP_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Starts the PWM signal generation.
  */
extern void Bsp_Tim_Init(void);
/**
  * @brief  Set the BMI088 Heat_Power TIM Capture Compare Register value.
  */
extern void Heat_Power_Control(uint16_t compare);

#endif //BSP_TIM_H
