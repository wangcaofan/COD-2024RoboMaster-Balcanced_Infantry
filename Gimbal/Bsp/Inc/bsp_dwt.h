/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_dwt.h
  * @brief          : bsp DWT functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_DWT_H
#define BSP_DWT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the DWT timer.
 */
typedef struct
{
  uint32_t s;
  uint16_t ms;
  uint16_t us;
} DWT_Time_Typedef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the DWT according to writing the specified data 
  *        to the internal configuration registers.
  */
extern void BSP_DWT_Init(uint32_t CPU_Freq_mHz);
/**
  * @brief get the DWT timedelta.
  */
extern float DWT_GetDeltaT(uint32_t *cnt_last);
/**
  * @brief get the DWT timedelta.
  */
extern double DWT_GetDeltaT64(uint32_t *cnt_last);
/**
  * @brief get the DWT second timeline.
  */
extern float DWT_GetTimeline_s(void);
/**
  * @brief get the DWT millisecond timeline.
  */
extern float DWT_GetTimeline_ms(void);
/**
  * @brief get the DWT microsecond timeline.
  */
extern uint64_t DWT_GetTimeline_us(void);
/**
  * @brief DWT Delay.
  */
extern void DWT_Delay(float Delay);
/**
  * @brief Update the DWT timeline.
  */
extern void DWT_SysTimeUpdate(void);

#endif //BSP_DWT_H

