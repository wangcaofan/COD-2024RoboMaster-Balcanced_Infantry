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

/* Includes ------------------------------------------------------------------*/
#include "lpf.h"

/**
  * @brief Initializes the first order lowpass filter according to the specified parameters in the
  *         LowPassFilter1p_Info_TypeDef.
  * @param lpf: pointer to an LowPassFilter1p_Info_TypeDef structure that
  *         contains the information  for the first order lowpass filter.
  * @param alpha: filter coefficient
  * @param frame_period: frame perood
  * @retval none
  */
float sign(float input){
   return (input>0.0f) - (input<0.0f);

}
void Tracking_Differentiator_Init(Tracking_Differentiator_Info_TypeDef *TD,float r,float h){
   TD->Input    = 0; 
   TD->Output   = 0;
   TD->d_Output = 0;  
	 TD->fh = 0;
   TD->r = r;
   TD->h = h;
}
void Tracking_Differentiator_Update(Tracking_Differentiator_Info_TypeDef *TD,float Input){
 TD->fh = -TD->r*TD->r*(TD->Output-Input) -2*TD->r*TD->d_Output;
 TD->Output +=  TD->h* TD->d_Output;
 TD->d_Output +=  TD->fh * TD->h;

}
void LowPassFilter1p_Init(LowPassFilter1p_Info_TypeDef *lpf,float alpha,float frame_period)
{
  lpf->alpha = alpha;
  lpf->frame_period = frame_period;
  lpf->input = 0;
  lpf->output = 0;
}
//------------------------------------------------------------------------------


/**
  * @brief Update the first order lowpass filter according to the specified parameters in the
  *         LowPassFilter1p_Info_TypeDef.
  * @param kf: pointer to an LowPassFilter1p_Info_TypeDef structure that
  *         contains the information  for the first order lowpass filter.
  * @param input: the filter input
  * @retval lowpass filter output
  */
float LowPassFilter1p_Update(LowPassFilter1p_Info_TypeDef *lpf,float input)
{
  lpf->input = input;

  if(lpf->Initialized == false)
  {
    lpf->output = lpf->input;
    lpf->Initialized = true;
  }

  lpf->output = lpf->alpha / (lpf->alpha + lpf->frame_period) * lpf->output 
              + lpf->frame_period / (lpf->alpha + lpf->frame_period) * lpf->input;

  return lpf->output;
}
//------------------------------------------------------------------------------


/**
  * @brief Initializes the Second order lowpass filter according to the specified parameters in the
  *         LowPassFilter2p_Info_TypeDef.
  * @param lpf: pointer to an LowPassFilter2p_Info_TypeDef structure that
  *         contains the information  for the Second order lowpass filter.
  * @param alpha: the floating-point array of filter coefficient
  * @param frame_period: frame perood
  * @retval none
  */
void LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *lpf,float alpha[3])
{
  memcpy(lpf->alpha,alpha,sizeof(lpf->alpha));
  lpf->input = 0;
  memset(lpf->output,0,sizeof(lpf->output));
}
//------------------------------------------------------------------------------


/**
  * @brief Update the Second order lowpass filter according to the specified parameters in the
  *         LowPassFilter2p_Info_TypeDef.
  * @param kf: pointer to an LowPassFilter2p_Info_TypeDef structure that
  *         contains the information  for the Second order lowpass filter.
  * @param input: the filter input
  * @retval lowpass filter output
  */
float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *lpf,float input)
{
	lpf->input = input;
  
  if(lpf->Initialized == false)
  {
    lpf->output[0] = lpf->input;
    lpf->output[1] = lpf->input;
    lpf->output[2] = lpf->input;
    lpf->Initialized = true;
  }
  
	lpf->output[0] = lpf->output[1];
	lpf->output[1] = lpf->output[2];
  lpf->output[2] = lpf->alpha[1] * lpf->output[0] + lpf->alpha[0] * lpf->output[1] + lpf->alpha[2] * lpf->input;

	return lpf->output[2];
}
//------------------------------------------------------------------------------

