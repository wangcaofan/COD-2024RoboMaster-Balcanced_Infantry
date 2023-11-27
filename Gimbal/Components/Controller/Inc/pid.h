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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief macro definition of the VAL_LIMIT that restricts the value of the specified variable.
 * @param x: the specified variable
 * @param min: the minimum value of the specified variable
 * @param max: the maximum value of the specified variable
 * @retval none
 */
#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

/**
 * @brief macro definition of the number of pid parameters
*/
#define PID_PARAMETER_NUM 6							

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the Error status for the pid controller.
 */
typedef enum
{
    PID_ERROR_NONE = 0x00U,        /*!< No error */
    PID_FAILED_INIT = 0x01U,        /*!< Initialization failed */
		PID_CALC_NANINF = 0x02U,      /*!< Not-a-number (NaN) or infinity is generated */
    PID_Status_NUM,
}PID_Status_e;

/**
 * @brief typedef enum that contains the type for the pid controller.
 */
typedef enum
{
		PID_Type_None = 0x00U,         /*!< No Type */
		PID_POSITION = 0x01U,          /*!< position pid */
		PID_VELOCITY = 0x02U,          /*!< velocity pid */
    PID_TYPE_NUM,
}PID_Type_e;

/**
 * @brief typedef structure that contains the information for the pid Error handler.
 */
typedef struct
{
    uint16_t ErrorCount;    /*!< Error status judgment count */
    PID_Status_e Status;    /*!< Error Status */
}PID_ErrorHandler_Typedef;

/**
 * @brief typedef structure that contains the parameters for the pid controller.
 */
typedef struct
{
    float kp;             /*!< Proportional Gain */
    float ki;             /*!< Integral Gain */
    float kd;             /*!< Derivative Gain */

		float Deadband;       /*!< Response Dead Zone */
    float limitIntegral;  /*!< Integral Limit */
    float limitOutput;    /*!< Output Limit */
}PID_Parameter_Typedef;

/**
 * @brief typedef structure that contains the information for the pid controller.
 */
typedef struct _PID_TypeDef
{
		PID_Type_e type;    /*!< type of pid */
	
		float target;       /*!< target value */
		float measure;      /*!< measurement value */
	
    float Err[3];       /*!< Error;previous Error;previous previous Error */
		float Integral;     /*!< Integral */

    float Pout;         /*!< Proportional Output */
    float Iout;         /*!< Integral Output */
    float Dout;         /*!< Derivative Output */
    float Output;       /*!< PID Output */
	
		PID_Parameter_Typedef param;            /*!< structure that contains the parameters */
    PID_ErrorHandler_Typedef ERRORHandler;  /*!< structure that contains the pid Error handler. */

    /**
     * @brief pointer for the function that Initializes the pid parameters.
     * @param pid: pointer to a _PID_TypeDef structure that
     *         contains the information for the PID controller.
     * @param para: pointer to a floating-point array that
     *         contains the parameters for the PID controller.
     * @retval pid error status
     */
    PID_Status_e (*PID_Param_Init)(struct _PID_TypeDef *pid,float *para);

    /**
     * @brief pointer for the function that Clear the pid Calculation.
     * @param pid: pointer to a _PID_TypeDef structure that
     *         contains the information for the PID controller.
     * @retval none
     */
		void (*PID_Calc_Clear)(struct _PID_TypeDef *pid);
				
}PID_Info_TypeDef;


/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Initializes the PID Controller.
 */
extern void PID_Init(PID_Info_TypeDef *Pid,PID_Type_e type,float para[PID_PARAMETER_NUM]);
/**
  * @brief  Caculate the PID Controller
  */
extern float f_PID_Calculate(PID_Info_TypeDef *Pid, float target,float measure);

#endif //CONTROLLER_PID_H


