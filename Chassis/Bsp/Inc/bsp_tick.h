/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tick.c
  * @brief          : HAL delay functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : use the TIM2 as the HAL TimeBase
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_TICK_H
#define BSP_TICK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  microsecond delay
  */
extern void Delay_us(uint32_t us);
/**
  * @brief  millisecond delay
  */
extern void Delay_ms(uint32_t ms);

#endif //BSP_TICK_H
