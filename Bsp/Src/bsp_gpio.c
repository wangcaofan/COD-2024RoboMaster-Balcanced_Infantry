/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_gpio.c
  * @brief          : bsp gpio functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_gpio.h"
#include "main.h"

/**
  * @brief RESET the BMI088_ACCEL_NS
  * @note GPIO_x: GPIOA
  * @note GPIO_PIN_x: GPIO_PIN_4
  */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port,CS1_ACCEL_Pin,GPIO_PIN_RESET);
}
//------------------------------------------------------------------------------


/**
  * @brief SET the BMI088_ACCEL_NS
  * @note GPIO_x: GPIOA
  * @note GPIO_PIN_x: GPIO_PIN_4
  */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port,CS1_ACCEL_Pin,GPIO_PIN_SET);
}
//------------------------------------------------------------------------------


/**
  * @brief RESET the BMI088_GYRO_NS
  * @note GPIO_x: GPIOB
  * @note GPIO_PIN_x: GPIO_PIN_0
  */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,CS1_GYRO_Pin,GPIO_PIN_RESET);
}
//------------------------------------------------------------------------------


/**
  * @brief SET the BMI088_GYRO_NS
  * @note GPIO_x: GPIOB
  * @note GPIO_PIN_x: GPIO_PIN_0
  */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,CS1_GYRO_Pin,GPIO_PIN_SET);
}
//------------------------------------------------------------------------------




