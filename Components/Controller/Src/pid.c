/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : pid functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"

/**
 * @brief Initializes the PID Parameters.
 * @param pid: pointer to a PID_Info_TypeDef structure that
 *         contains the information for the PID controller.
 * @param para: pointer to a floating-point array that
 *         contains the parameters for the PID controller.
 * @retval pid error status
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *Pid,float para[PID_PARAMETER_NUM])
{
    /* judge the pointer of PID Parameters */
    if(Pid->type == PID_Type_None || para == NULL)
    {
      return PID_FAILED_INIT;
    }

    /* Initializes the pid Parameters ------------------*/
    Pid->param.kp = para[0];
    Pid->param.ki = para[1];
    Pid->param.kd = para[2];
    Pid->param.Deadband = para[3];
    Pid->param.limitIntegral = para[4];
    Pid->param.limitOutput = para[5];

    /* clear the pid error judgement count */
    Pid->ERRORHandler.ErrorCount = 0;

    return PID_ERROR_NONE;
}
//------------------------------------------------------------------------------


/**
 * @brief Clear the Pid Calculation.
 * @param pid: pointer to a PID_Info_TypeDef structure that
 *         contains the information for the PID controller.
 * @retval none
 */
static void PID_Calc_Clear(PID_Info_TypeDef *Pid)
{
	memset(Pid->Err,0,sizeof(Pid->Err));
	Pid->Integral = 0;
		
	Pid->Pout = 0;
	Pid->Iout = 0;
	Pid->Dout = 0;
	Pid->Output = 0;
}
//------------------------------------------------------------------------------


/**
 * @brief Initializes the PID Controller.
 * @param pid: pointer to a PID_Info_TypeDef structure that
 *         contains the information for the PID controller.
 * @param type: type of pid controller
 * @param para: pointer to a floating-point array that
 *         contains the parameters for the PID controller.
 * @retval pid error status
 */
void PID_Init(PID_Info_TypeDef *Pid,PID_Type_e type,float para[PID_PARAMETER_NUM])
{
		Pid->type = type;

		Pid->PID_Calc_Clear = PID_Calc_Clear;
    Pid->PID_Param_Init = PID_Param_Init;

		Pid->PID_Calc_Clear(Pid);
    Pid->ERRORHandler.Status = Pid->PID_Param_Init(Pid, para);
}
//------------------------------------------------------------------------------


/**
  * @brief  Judge the pid error status
  * @param pid: pointer to a PID_Info_TypeDef structure that
  *         contains the information for the PID controller.
  * @retval None
  */
static void PID_ErrorHandle(PID_Info_TypeDef *Pid)
{
		/* Judge NAN/INF */
		if(isnan(Pid->Output) == true || isinf(Pid->Output)==true)
		{
				Pid->ERRORHandler.Status = PID_CALC_NANINF;
		}
}
//------------------------------------------------------------------------------

/**
  * @brief  Caculate the PID Controller
  * @param  *pid pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID. 
  * @param  Target  Target for the pid controller
  * @param  Measure Measure for the pid controller
  * @retval the Pid Output
  */
float f_PID_Calculate(PID_Info_TypeDef *Pid, float target,float measure)
{		
  /* update the pid error status */
  PID_ErrorHandle(Pid);
  if(Pid->ERRORHandler.Status != PID_ERROR_NONE)
  {
    Pid->PID_Calc_Clear(Pid);
    return 0;
  }
  
  /* update the target/measure */
  Pid->target = target;
  Pid->measure = measure;

  /* update the error */
	Pid->Err[2] = Pid->Err[1];
	Pid->Err[1] = Pid->Err[0];
	Pid->Err[0] = Pid->target - Pid->measure;
		
  if(fabsf(Pid->Err[0]) >= Pid->param.Deadband)
  {
		/* update the pid controller output */
		if(Pid->type == PID_POSITION)
		{
      /* Update the PID Integral */
      if(Pid->param.ki != 0)
        Pid->Integral += Pid->Err[0];
      else
        Pid->Integral = 0;

      VAL_LIMIT(Pid->Integral,-Pid->param.limitIntegral,Pid->param.limitIntegral);
      
      /* Update the Proportional Output,Integral Output,Derivative Output */
      Pid->Pout = Pid->param.kp * Pid->Err[0];
      Pid->Iout = Pid->param.ki * Pid->Integral;
      Pid->Dout = Pid->param.kd * (Pid->Err[0] - Pid->Err[1]);
      
      /* update the PID output */
      Pid->Output = Pid->Pout + Pid->Iout + Pid->Dout;
      VAL_LIMIT(Pid->Output,-Pid->param.limitOutput,Pid->param.limitOutput);
		}
		else if(Pid->type == PID_VELOCITY)
		{
      /* Update the Proportional Output,Integral Output,Derivative Output */
      Pid->Pout = Pid->param.kp * (Pid->Err[0] - Pid->Err[1]);
      Pid->Iout = Pid->param.ki * (Pid->Err[0]);
      Pid->Dout = Pid->param.kd * (Pid->Err[0] - 2.f*Pid->Err[1] + Pid->Err[2]);

      /* update the PID output */
      Pid->Output += Pid->Pout + Pid->Iout + Pid->Dout;
      VAL_LIMIT(Pid->Output,-Pid->param.limitOutput,Pid->param.limitOutput);
		}
  }

  return Pid->Output;
}
//------------------------------------------------------------------------------

