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

/* Includes ------------------------------------------------------------------*/
#include "bsp_mcu.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "bsp_uart.h"
#include "bsp_dwt.h"
#include "bmi088.h"
#include "usb_device.h"

/**
  * @brief Initializes the MCU.
  */
void MCU_Init(void)
{
  /* ----------------------- BSP Init ----------------------- */
  Bsp_Tim_Init();
  BSP_CAN_Init();
  BSP_USART_Init();
	MX_USB_DEVICE_Init();

  /* ----------------------- Device Init ----------------------- */
  BMI088_Init();
}
//------------------------------------------------------------------------------

