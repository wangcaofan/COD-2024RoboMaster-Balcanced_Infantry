/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lowpass_filter.c
  * @brief          : lowpass filter 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information  for the first order lowpass filter.
 */
typedef struct
{
    bool Initialized;     /*!< init flag */
    float input;          /*!< input value */
    float output;         /*!< output value */
    float alpha;          /*!< filter coefficient */
    float frame_period;   /*!< frame perood */
}LowPassFilter1p_Info_TypeDef;

/**
 * @brief typedef structure that contains the information for the second order lowpass filter.
 */
typedef struct 
{
    bool Initialized;  /*!< init flag */
    float input;       /*!< input value */
    float output[3];   /*!< output value */
    float alpha[3];    /*!< filter coefficient */
}LowPassFilter2p_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the first order lowpass filter according to the specified parameters in the
  *         LowPassFilter1p_Info_TypeDef.
  */
extern void LowPassFilter1p_Init(LowPassFilter1p_Info_TypeDef *lpf,float alpha,float frame_period);
/**
  * @brief Update the first order lowpass filter according to the specified parameters in the
  *         LowPassFilter1p_Info_TypeDef.
  */
extern float LowPassFilter1p_Update(LowPassFilter1p_Info_TypeDef *lpf,float input);
/**
  * @brief Initializes the Second order lowpass filter according to the specified parameters in the
  *         LowPassFilter2p_Info_TypeDef.
  */
extern void LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *lpf,float alpha[3]);
/**
  * @brief Update the Second order lowpass filter according to the specified parameters in the
  *         LowPassFilter2p_Info_TypeDef.
  */
extern float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *lpf,float input);

#endif //LOWPASS_FILTER_H
