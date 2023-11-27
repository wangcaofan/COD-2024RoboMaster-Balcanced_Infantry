/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_mcu.c
  * @brief          : MCU peripheral initialization functions
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_MCU_H
#define BSP_MCU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the MCU.
  */
extern void MCU_Init(void);

#endif //BSP_MCU_H

