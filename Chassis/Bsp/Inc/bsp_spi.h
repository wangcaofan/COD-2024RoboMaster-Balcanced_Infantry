/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_spi.c
  * @brief          : bsp spi functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_SPI_H
#define BSP_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief returns the spi receive data after transmiting the specified data 
  */
extern uint8_t BMI088_Read_Write_Byte(uint8_t txdata);

#endif //BSP_SPI_H
