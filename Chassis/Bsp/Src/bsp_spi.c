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

/* Includes ------------------------------------------------------------------*/
#include "bsp_spi.h"

#include "spi.h"

/**
  * @brief returns the spi receive data after transmiting the specified data 
  * @param txdata: the specified data
  * @retval the spi receive data
  */
uint8_t BMI088_Read_Write_Byte(uint8_t txdata)
{
    uint8_t rxdata = 0;

		HAL_SPI_TransmitReceive(&hspi1,&txdata,&rxdata,1,1000);

    return rxdata;
}
//------------------------------------------------------------------------------



