/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : kalman.c
  * @brief          : Kalman filter 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  * @note kalman filter formula:
  *       1.xhatminus = A xhat(k-1) + B u(k-1)
  *       2.Pminus = A P(k-1) AT + Q
  *       3.K = H·Pminus / (H·Pminus·HT + R)
  *       4.xhat = xhatminus + K(k)·(z(k) - H·xhatminus)
  *       5.P = (I - K(k)·H)·Pminus
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "kalman.h"

/**
  * @brief Initializes the kalman filter according to the specified parameters in the
  *         KalmanFilter_Info_TypeDef.
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @param xhatSize: state vector dimension
  * @param uSize: control vector dimension
  * @param zSize: measurement vector dimension
  * @retval none
  */
void Kalman_Filter_Init(KalmanFilter_Info_TypeDef *kf,uint8_t xhatSize,uint8_t uSize,uint8_t zSize)
{
    /* Update the size of float/double */
    kf->sizeof_float = sizeof(float);
    kf->sizeof_double = sizeof(double);

    /* judge the length of vector */
    if(xhatSize == 0 || zSize == 0)
    {
        kf->MatStatus = ARM_MATH_LENGTH_ERROR;  
    }
    
    /* Initializes the state vector dimension */
    kf->xhatSize = xhatSize;

     /* Initializes the control vector dimension */
    kf->uSize = uSize;

    /**< Initializes the measurement vector dimension */      
    kf->zSize = zSize;

    /* Initializes the ChiSquare matrix */
    memset(kf->ChiSquareTest.ChiSquare_Data,0,sizeof(kf->ChiSquareTest.ChiSquare_Data));
    Matrix_Init(&kf->ChiSquareTest.ChiSquare_Matrix, 1, 1, (float *)kf->ChiSquareTest.ChiSquare_Data);

    /* Initializes the external measure vector */
    kf->MeasuredVector = (float *)user_malloc(kf->sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, kf->sizeof_float * zSize);

    /* Initializes the xhat */
    kf->Data.xhat = (float *)user_malloc(kf->sizeof_float * xhatSize);
    memset(kf->Data.xhat, 0, kf->sizeof_float * xhatSize);
    Matrix_Init(&kf->Mat.xhat, kf->xhatSize, 1, (float *)kf->Data.xhat);

    /* Initializes the xhatminus */
    kf->Data.xhatminus = (float *)user_malloc(kf->sizeof_float * xhatSize);
    memset(kf->Data.xhatminus, 0, kf->sizeof_float * xhatSize);
    Matrix_Init(&kf->Mat.xhatminus, kf->xhatSize, 1, (float *)kf->Data.xhatminus);

    /* Initializes the measurement vector */
    kf->Data.z = (float *)user_malloc(kf->sizeof_float * zSize);
    memset(kf->Data.z, 0, kf->sizeof_float * zSize);
    Matrix_Init(&kf->Mat.z, kf->zSize, 1, (float *)kf->Data.z);

    if (kf->uSize != 0)
    {
				/* Initializes the external control vector */
				kf->ControlVector = (float *)user_malloc(kf->sizeof_float * uSize);
				memset(kf->ControlVector, 0, kf->sizeof_float * uSize);

        /* Initializes the control vector */ 
        kf->Data.u = (float *)user_malloc(kf->sizeof_float * uSize);
        memset(kf->Data.u, 0, kf->sizeof_float * uSize);
        Matrix_Init(&kf->Mat.u, kf->uSize, 1, (float *)kf->Data.u);

        /* Initializes the control Matrix */  
        kf->Data.B = (float *)user_malloc(kf->sizeof_float * xhatSize * uSize);
        memset(kf->Data.B, 0, kf->sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->Mat.B, kf->xhatSize, kf->uSize, (float *)kf->Data.B);
    }

    /* Initializes the state transition Matrix */ 
    kf->Data.A = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.A, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.A, kf->xhatSize, kf->xhatSize, (float *)kf->Data.A);

    kf->Data.AT = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.AT, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.AT, kf->xhatSize, kf->xhatSize, (float *)kf->Data.AT);

    /* Initializes the measurement Matrix */ 
    kf->Data.H = (float *)user_malloc(kf->sizeof_float * zSize * xhatSize);
    memset(kf->Data.H, 0, kf->sizeof_float * zSize * xhatSize);
    Matrix_Init(&kf->Mat.H, kf->zSize, kf->xhatSize, (float *)kf->Data.H);

    kf->Data.HT = (float *)user_malloc(kf->sizeof_float * xhatSize * zSize);
    memset(kf->Data.HT, 0, kf->sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->Mat.HT, kf->xhatSize, kf->zSize, (float *)kf->Data.HT);

    /* Initializes the posteriori covariance Matrix */
    kf->Data.P = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.P, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.P, kf->xhatSize, kf->xhatSize, (float *)kf->Data.P);

    /* Initializes the priori covariance Matrix */
    kf->Data.Pminus = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.Pminus, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Data.Pminus);

    /* Initializes the process noise covariance Matrix */  
    kf->Data.Q = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.Q, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.Q, kf->xhatSize, kf->xhatSize, (float *)kf->Data.Q);

    /* Initializes the measurement noise covariance Matrix */
    kf->Data.R = (float *)user_malloc(kf->sizeof_float * zSize * zSize);
    memset(kf->Data.R, 0, kf->sizeof_float * zSize * zSize);
    Matrix_Init(&kf->Mat.R, kf->zSize, kf->zSize, (float *)kf->Data.R);

    /* Initializes the kalman gain */  
    kf->Data.K = (float *)user_malloc(kf->sizeof_float * xhatSize * zSize);
    memset(kf->Data.K, 0, kf->sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->Mat.K, kf->xhatSize, kf->zSize, (float *)kf->Data.K);

    /* Initializes the K_denominator (K_denominator = H Pminus HT + R) */  
    kf->Data.K_denominator = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Data.K_denominator, 0, kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    Matrix_Init(&kf->Mat.K_denominator, kf->xhatSize, kf->xhatSize, (float *)kf->Data.K_denominator);

    /* Initializes the calculate cache Matrix */
    kf->Data.cache_matrix[0] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Data.cache_matrix[0],0,kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    Matrix_Init(&kf->Mat.cache_matrix[0], kf->xhatSize, kf->xhatSize, (float *)kf->Data.cache_matrix[0]);

    kf->Data.cache_matrix[1] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Data.cache_matrix[1],0,kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    Matrix_Init(&kf->Mat.cache_matrix[1], kf->xhatSize, kf->xhatSize, (float *)kf->Data.cache_matrix[1]);

    /* Initializes the calculate cache vector */
    kf->Data.cache_vector[0] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize);
    memset(kf->Data.cache_vector[0],0,kf->sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->Mat.cache_vector[0], kf->xhatSize, 1, (float *)kf->Data.cache_vector[0]);

    kf->Data.cache_vector[1] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize);
    memset(kf->Data.cache_vector[1],0,kf->sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->Mat.cache_vector[1], kf->xhatSize, 1, (float *)kf->Data.cache_vector[1]);

    /* Initializes the filter output */
    kf->Output = (float *)user_malloc(kf->sizeof_float * xhatSize);
    memset(kf->Output, 0, kf->sizeof_float * xhatSize);
}
//------------------------------------------------------------------------------


/**
  * @brief Update the Measuerment Information
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @retval none
  */
static void Kalman_Filter_Measurement_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* update the measuerment vector from the external measuerment vector */
    memcpy(kf->Data.z, kf->MeasuredVector, kf->sizeof_float * kf->zSize);

    /* clear the external measuerment vector */
    memset(kf->MeasuredVector, 0, kf->sizeof_float * kf->zSize);

    if(kf->uSize > 0)
    {
      /* update the control vector from the external control vector */
      memcpy(kf->Data.u, kf->ControlVector, kf->sizeof_float * kf->uSize);
    }
}
//------------------------------------------------------------------------------


/**
  * @brief Update the Priori EstiMate
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note xhatminus = A xhat(k-1) + B u(k-1)
  * @retval none
  */
static void Kalman_Filter_xhatminus_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* skip this step,replaced with user function */
    if(kf->SkipStep1 == 1)
    {
      return;
    }

    if(kf->uSize > 0)
    {
        /* cache_vector[0] = A xhat(k-1) */ 
        kf->Mat.cache_vector[0].numRows = kf->xhatSize;
        kf->Mat.cache_vector[0].numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->Mat.A, &kf->Mat.xhat, &kf->Mat.cache_vector[0]);   

        /* cache_vector[1] = B u(k-1) */ 
        kf->Mat.cache_vector[0].numRows = kf->xhatSize;
        kf->Mat.cache_vector[0].numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->Mat.B, &kf->Mat.u, &kf->Mat.cache_vector[1]);    

        /* xhatminus = A xhat(k-1) + B u(k-1) */
        kf->MatStatus = Matrix_Add(&kf->Mat.cache_vector[0], &kf->Mat.cache_vector[1], &kf->Mat.xhatminus);   
    }
    /* lack of control vector */
    else
    {
        /* xhatminus = A xhat(k-1) */
        kf->MatStatus = Matrix_Multiply(&kf->Mat.A, &kf->Mat.xhat, &kf->Mat.xhatminus);   
    }
}
//------------------------------------------------------------------------------

/**
  * @brief Update the Priori Covariance Matrix
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note Pminus = A P(k-1) AT + Q
  * @retval none
  */
static void Kalman_Filter_Pminus_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* skip this step,replaced with user function */
    if(kf->SkipStep2 == 1)
    {
      return;
    }

    /* AT */
    kf->MatStatus = Matrix_Transpose(&kf->Mat.A, &kf->Mat.AT); 

    /* Pminus = A P(k-1) */ 
    kf->MatStatus = Matrix_Multiply(&kf->Mat.A, &kf->Mat.P, &kf->Mat.Pminus); 

    /* cache_matrix[0] = A P(k-1) AT */ 
    kf->Mat.cache_matrix[0].numRows = kf->Mat.Pminus.numRows;
    kf->Mat.cache_matrix[0].numCols = kf->Mat.AT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.Pminus, &kf->Mat.AT, &kf->Mat.cache_matrix[0]); 

    /* Pminus = A P(k-1) AT + Q */
    kf->MatStatus = Matrix_Add(&kf->Mat.cache_matrix[0], &kf->Mat.Q, &kf->Mat.Pminus);  
}
//------------------------------------------------------------------------------

/**
  * @brief Update the Kalman Gain
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note K = H·Pminus / (H·Pminus·HT + R)
  * @retval none
  */
static void Kalman_Filter_K_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* skip this step,replaced with user function */
    if(kf->SkipStep3 == 1)
    {
      return;
    }

    /* HT */
    kf->MatStatus = Matrix_Transpose(&kf->Mat.H, &kf->Mat.HT); 

    /* cache_matrix[0] = H·Pminus */
    kf->Mat.cache_matrix[0].numRows = kf->Mat.H.numRows;
    kf->Mat.cache_matrix[0].numCols = kf->Mat.Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.H, &kf->Mat.Pminus, &kf->Mat.cache_matrix[0]); 

    /* cache_matrix[1] = H·Pminus·HT */
    kf->Mat.cache_matrix[1].numRows = kf->Mat.cache_matrix[0].numRows;
    kf->Mat.cache_matrix[1].numCols = kf->Mat.HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_matrix[0], &kf->Mat.HT, &kf->Mat.cache_matrix[1]);  

    /* K_denominator = H·Pminus·HT + R */
    kf->Mat.K_denominator.numRows = kf->Mat.R.numRows;
    kf->Mat.K_denominator.numCols = kf->Mat.R.numCols;
    kf->MatStatus = Matrix_Add(&kf->Mat.cache_matrix[1], &kf->Mat.R, &kf->Mat.K_denominator); 

    /* cache_matrix[1] = inverse(H·Pminus·HT + R) */
    kf->MatStatus = Matrix_Inverse(&kf->Mat.K_denominator, &kf->Mat.cache_matrix[1]);

    /* cache_matrix[0] = Pminus·HT */
    kf->Mat.cache_matrix[0].numRows = kf->Mat.Pminus.numRows;
    kf->Mat.cache_matrix[0].numCols = kf->Mat.HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.Pminus, &kf->Mat.HT, &kf->Mat.cache_matrix[0]);

    /* K = H·Pminus / (H·Pminus·HT + R) */
    kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_matrix[0], &kf->Mat.cache_matrix[1], &kf->Mat.K);
}
//------------------------------------------------------------------------------

/**
  * @brief Update the Posteriori EstiMate
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note xhat = xhatminus + K(k)·(z(k) - H·xhatminus)
  * @retval none
  */
static void Kalman_Filter_xhat_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* skip this step,replaced with user function */
    if(kf->SkipStep4 == 1)
    {
      return;
    }

    /* cache_vector[0] = H xhatminus */
    kf->Mat.cache_vector[0].numRows = kf->Mat.H.numRows;
    kf->Mat.cache_vector[0].numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.H, &kf->Mat.xhatminus, &kf->Mat.cache_vector[0]);

    /* cache_vector[1] = z(k) - H·xhatminus */
    kf->Mat.cache_vector[1].numRows = kf->Mat.z.numRows;
    kf->Mat.cache_vector[1].numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->Mat.z, &kf->Mat.cache_vector[0], &kf->Mat.cache_vector[1]); 

    /* cache_vector[0] = K(k)·(z(k) - H·xhatminus) */
    kf->Mat.cache_vector[0].numRows = kf->Mat.K.numRows;
    kf->Mat.cache_vector[0].numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.K, &kf->Mat.cache_vector[1], &kf->Mat.cache_vector[0]);

    /* xhat = xhatminus + K(k)·(z(k) - H·xhatminus) */
    kf->MatStatus = Matrix_Add(&kf->Mat.xhatminus, &kf->Mat.cache_vector[0], &kf->Mat.xhat); 
}
//------------------------------------------------------------------------------
/**
  * @brief  Update the Posteriori Covariance Matrix
  * @param  kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note   P = (I - K(k)·H)·Pminus
  * @retval none
  */
static void Kalman_Filter_P_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* skip this step,replaced with user function */
    if(kf->SkipStep5 == 1)
    {
      return;
    }

    /* cache_vector[0] = K(k)·H */
    kf->Mat.cache_matrix[0].numRows = kf->Mat.K.numRows;
    kf->Mat.cache_matrix[0].numCols = kf->Mat.H.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.K, &kf->Mat.H, &kf->Mat.cache_matrix[0]);

    /* cache_vector[1] = K(k)·H·Pminus */
    kf->Mat.cache_matrix[1].numRows = kf->Mat.cache_matrix[0].numRows;
    kf->Mat.cache_matrix[1].numCols = kf->Mat.Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_matrix[0], &kf->Mat.Pminus, &kf->Mat.cache_matrix[1]);
		
    /* P = (I - K(k)·H)·Pminus */
    kf->MatStatus = Matrix_Subtract(&kf->Mat.Pminus, &kf->Mat.cache_matrix[1], &kf->Mat.P); 
}
//------------------------------------------------------------------------------

/**
  * @brief Update the Kalman Filter according to the specified parameters in the
  *         KalmanFilter_Info_TypeDef.
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @retval pointer of kalman filter output
  * @note kalman filter formula:
  *       1.xhatminus = A xhat(k-1) + B u(k-1)
  *       2.Pminus = A P(k-1) AT + Q
  *       3.K = H·Pminus / (H·Pminus·HT + R)
  *       4.xhat = xhatminus + K(k)·(z(k) - H·xhatminus)
  *       5.P = (I - K(k)·H)·Pminus
  */
float *Kalman_Filter_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* Update the Measuerment Information */
    Kalman_Filter_Measurement_Update(kf);
    /* User Function 0 */
    if(kf->User_Function0 != NULL)
    {
      kf->User_Function0(kf);
    }

    /* Update the Priori EstiMate */
    Kalman_Filter_xhatminus_Update(kf);
    /* User Function 1 */
    if(kf->User_Function1 != NULL)
    {
      kf->User_Function1(kf);
    }

    /* Update the Priori Covariance Matrix */
    Kalman_Filter_Pminus_Update(kf);
    /* User Function 2 */
    if(kf->User_Function2 != NULL)
    {
      kf->User_Function2(kf);
    }
    /* Update the kalman filter */
    Kalman_Filter_K_Update(kf);
    /* User Function 3 */
    if(kf->User_Function3 != NULL)
    {
      kf->User_Function3(kf);
    }

    /* Update the Posteriori EstiMate */
    Kalman_Filter_xhat_Update(kf);
    /* User Function 4 */
    if(kf->User_Function4 != NULL)
    {
      kf->User_Function4(kf);
    }

    /* Update the Posteriori Covariance Matrix */
    Kalman_Filter_P_Update(kf);
    /* User Function 5 */
    if(kf->User_Function5 != NULL)
    {
      kf->User_Function5(kf);
    }

    /* User Function 6 */
    if(kf->User_Function6 != NULL)
    {
      kf->User_Function6(kf);
    }

    /* Update the kalman filter output */
    memcpy(kf->Output, kf->Data.xhat, kf->sizeof_float * kf->xhatSize);

    return kf->Output;
}
//------------------------------------------------------------------------------


