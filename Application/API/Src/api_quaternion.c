/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_quaternion.c
  * @brief          : quaternion fusion api
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  * @note           : see .\Docs\Quaternion.pdf
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "api_quaternion.h"
#include "math.h"
#include "pid.h"
#include "config.h"

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  fast calculate the inverse square root
  */
static float Fast_InverseSqrt(float);
/**
  * @brief Update the state transition matrix
  */
static void QuaternionEKF_A_Update(KalmanFilter_Info_TypeDef *);
/**
  * @brief Update the measurement matrix
  */
static void QuaternionEKF_H_Update(KalmanFilter_Info_TypeDef *);
/**
  * @brief Update the posteriori estimate matrix
  */
static void QuaternionEKF_xhat_Update(KalmanFilter_Info_TypeDef *);

/**
  * @brief Initializes the Quaternion EKF according to the specified parameters in the
  *         Quaternion_Info_Typedef.
  * @param quat: pointer to a Quaternion_Info_Typedef structure that
  *         contains the information  for the Quaternion EKF
  * @param alpha: pointer to the data of LPF2p filter coefficient
  * @param process_noise1/2: process noise covariance matrix
  * @param measure_noise: measurement noise covariance matrix
  * @param QuaternionEKF_A_Data: pointer to the Initialize data of state transition matrix
  * @param QuaternionEKF_P_Data: pointer to the Initialize data of posteriori covariance matrix
  * @retval none
  */
void QuaternionEKF_Init(Quaternion_Info_Typedef *quat,float process_noise1,float process_noise2,float measure_noise,float *QuaternionEKF_A_Data,float *QuaternionEKF_P_Data)
{
  /* Initializes the data of process/measurement noise covariance matrix */
  quat->Q1 = process_noise1;
  quat->Q2 = process_noise2;
  quat->R  = measure_noise;

  /* Initializes the pointer to the data of state transition/posteriori covariance matrix */
  quat->QuaternionEKF_A_Data = QuaternionEKF_A_Data;
  quat->QuaternionEKF_P_Data = QuaternionEKF_P_Data;

  /* Initializes the Extended kalman filter */
  Kalman_Filter_Init(&quat->QuaternionEKF,6,0,3);

  /* Initializes the chi square test */
  quat->QuaternionEKF.ChiSquareTest.TestFlag = false;
  quat->QuaternionEKF.ChiSquareTest.result = false;
  quat->QuaternionEKF.ChiSquareTest.ChiSquareTestThresholds = 1e-8f;
  quat->QuaternionEKF.ChiSquareTest.ChiSquareCnt = 0;

  /* Initializes the position */
  quat->QuaternionEKF.Data.xhat[0] = 1.f;
  quat->QuaternionEKF.Data.xhat[1] = 0.f;
  quat->QuaternionEKF.Data.xhat[2] = 0.f;
  quat->QuaternionEKF.Data.xhat[3] = 0.f;

  quat->QuaternionEKF.User_Function1 = QuaternionEKF_A_Update;
  quat->QuaternionEKF.User_Function2 = QuaternionEKF_H_Update;
  quat->QuaternionEKF.User_Function3 = QuaternionEKF_xhat_Update;

  quat->QuaternionEKF.SkipStep3 = true;
  quat->QuaternionEKF.SkipStep4 = true;

  memcpy(quat->QuaternionEKF.Data.A,quat->QuaternionEKF_A_Data,quat->QuaternionEKF.sizeof_float * quat->QuaternionEKF.xhatSize * quat->QuaternionEKF.xhatSize);
  memcpy(quat->QuaternionEKF.Data.P,quat->QuaternionEKF_P_Data,quat->QuaternionEKF.sizeof_float * quat->QuaternionEKF.xhatSize * quat->QuaternionEKF.xhatSize);
}
//------------------------------------------------------------------------------

/**
  * @brief  Update the Extended Kalman Filter
  * @param quat: pointer to a Quaternion_Info_Typedef structure that
  *         contains the information  for the Quaternion EKF
  * @param gyro: pointer to the accel measurement data
  * @param accel: pointer to the gyro measurement data
  * @param dt: update cycle
  * @retval none
  */
void QuaternionEKF_Update(Quaternion_Info_Typedef *quat,float gyro[3],float accel[3],float dt)
{
  /* update cycle */
  quat->QuaternionEKF.dt = dt;

  /* Apply accelerometer feedback to gyroscope */
  quat->gyro[0] = gyro[0] - quat->deviate[0];
  quat->gyro[1] = gyro[1] - quat->deviate[1];
  quat->gyro[2] = gyro[2] - quat->deviate[2];

  quat->gyroInvNorm = Fast_InverseSqrt(quat->gyro[0]*quat->gyro[0]+quat->gyro[1]*quat->gyro[1]+quat->gyro[2]*quat->gyro[2]);

  /* Convert gyroscope to radians per second scaled by 0.5 */
  quat->halfgyrodt[0] = 0.5f * quat->gyro[0] * quat->QuaternionEKF.dt;
  quat->halfgyrodt[1] = 0.5f * quat->gyro[1] * quat->QuaternionEKF.dt;
  quat->halfgyrodt[2] = 0.5f * quat->gyro[2] * quat->QuaternionEKF.dt;

  /**
   * @brief A = \frac{\partial f}{\partial x}
   *        (1,        -halfgxdt,  -halfgydt,  -halfgzdt,)   0.5f*q1*dt,  0.5f*q2*dt
   *        (halfgxdt,  1,          halfgzdt,  -halfgydt,)  -0.5f*q0*dt,  0.5f*q3*dt
   *        (halfgydt, -halfgzdt,   1,          halfgxdt,)  -0.5f*q3*dt, -0.5f*q0*dt
   *        (halfgzdt,  halfgydt,  -halfgxdt,   1,       )   0.5f*q2*dt, -0.5f*q1*dt
   *        (0,         0,          0,          0,         1,            0 )
   *        (0,         0,          0,          0,         0,            1 )
   */
  memcpy(quat->QuaternionEKF.Data.A,quat->QuaternionEKF_A_Data,quat->QuaternionEKF.sizeof_float * quat->QuaternionEKF.xhatSize * quat->QuaternionEKF.xhatSize);

  quat->QuaternionEKF.Data.A[1]  = -quat->halfgyrodt[0];
  quat->QuaternionEKF.Data.A[2]  = -quat->halfgyrodt[1];
  quat->QuaternionEKF.Data.A[3]  = -quat->halfgyrodt[2];

  quat->QuaternionEKF.Data.A[6]  =  quat->halfgyrodt[0];
  quat->QuaternionEKF.Data.A[8]  =  quat->halfgyrodt[2];
  quat->QuaternionEKF.Data.A[9]  = -quat->halfgyrodt[1];

  quat->QuaternionEKF.Data.A[12] =  quat->halfgyrodt[1];
  quat->QuaternionEKF.Data.A[13] = -quat->halfgyrodt[2];
  quat->QuaternionEKF.Data.A[15] =  quat->halfgyrodt[0];

  quat->QuaternionEKF.Data.A[18] =  quat->halfgyrodt[2];
  quat->QuaternionEKF.Data.A[19] =  quat->halfgyrodt[1];
  quat->QuaternionEKF.Data.A[20] = -quat->halfgyrodt[0];
	
	memcpy(quat->accel,accel,sizeof(quat->accel));

  /* Calculate direction of gravity indicated by measurement */
  quat->accelInvNorm = Fast_InverseSqrt(quat->accel[0]*quat->accel[0]+quat->accel[1]*quat->accel[1]+quat->accel[2]*quat->accel[2]);
  quat->QuaternionEKF.MeasuredVector[0] = quat->accel[0] * quat->accelInvNorm;
  quat->QuaternionEKF.MeasuredVector[1] = quat->accel[1] * quat->accelInvNorm;
  quat->QuaternionEKF.MeasuredVector[2] = quat->accel[2] * quat->accelInvNorm;
	 
  /* chi square test */
  if(1.f/quat->gyroInvNorm < 0.3f && 1.f/quat->accelInvNorm  > (GravityAccel-0.5f) && 1.f/quat->accelInvNorm < (GravityAccel+0.5f))
	{
		quat->QuaternionEKF.ChiSquareTest.TestFlag = true;
  }
  else
  {
    quat->QuaternionEKF.ChiSquareTest.TestFlag = false;
  }

  /* update the process/measurement noise covariance matrix */
  quat->QuaternionEKF.Data.Q[0]  = quat->Q1 * quat->QuaternionEKF.dt;
  quat->QuaternionEKF.Data.Q[7]  = quat->Q1 * quat->QuaternionEKF.dt;
  quat->QuaternionEKF.Data.Q[14] = quat->Q1 * quat->QuaternionEKF.dt;
  quat->QuaternionEKF.Data.Q[21] = quat->Q1 * quat->QuaternionEKF.dt;
  quat->QuaternionEKF.Data.Q[28] = quat->Q2 * quat->QuaternionEKF.dt;
  quat->QuaternionEKF.Data.Q[35] = quat->Q2 * quat->QuaternionEKF.dt;
  quat->QuaternionEKF.Data.R[0]  = quat->R;
  quat->QuaternionEKF.Data.R[4]  = quat->R;
  quat->QuaternionEKF.Data.R[8]  = quat->R;

  /* update the kalman filter */
  Kalman_Filter_Update(&quat->QuaternionEKF);

  /* Update the fusion data */
  quat->quat[0]    = quat->QuaternionEKF.Output[0];
  quat->quat[1]    = quat->QuaternionEKF.Output[1];
  quat->quat[2]    = quat->QuaternionEKF.Output[2];
  quat->quat[3]    = quat->QuaternionEKF.Output[3];
  quat->deviate[0] = quat->QuaternionEKF.Output[4];
  quat->deviate[1] = quat->QuaternionEKF.Output[5];
  quat->deviate[2] = 0.f;
  
	/* Update the Euler angle in radians */
  quat->EulerAngle[0] = atan2f(2.f*(quat->quat[0]*quat->quat[3] + quat->quat[1]*quat->quat[2]), 2.f*(quat->quat[0]*quat->quat[0] + quat->quat[1]*quat->quat[1])-1.f);
  quat->EulerAngle[1] = asinf(-2.f*(quat->quat[1]*quat->quat[3] - quat->quat[0]*quat->quat[2]));
  quat->EulerAngle[2] = atan2f(2.f*(quat->quat[0]*quat->quat[1] + quat->quat[2]*quat->quat[3]), 2.f*(quat->quat[0]*quat->quat[0] + quat->quat[3]*quat->quat[3])-1.f);
}
//------------------------------------------------------------------------------

/**
  * @brief Update the state transition matrix
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @retval none
  */
static void QuaternionEKF_A_Update(KalmanFilter_Info_TypeDef *kf)
{
  /* Normalise quaternion */
  memset(kf->Data.cache_vector[0],0,kf->sizeof_float * kf->xhatSize);
  kf->Data.cache_vector[0][0] = Fast_InverseSqrt(kf->Data.xhatminus[0]*kf->Data.xhatminus[0] \
                                                +kf->Data.xhatminus[1]*kf->Data.xhatminus[1] \
                                                +kf->Data.xhatminus[2]*kf->Data.xhatminus[2] \
                                                +kf->Data.xhatminus[3]*kf->Data.xhatminus[3]);

  kf->Data.xhatminus[0] *= kf->Data.cache_vector[0][0];
  kf->Data.xhatminus[1] *= kf->Data.cache_vector[0][0];
  kf->Data.xhatminus[2] *= kf->Data.cache_vector[0][0];
  kf->Data.xhatminus[3] *= kf->Data.cache_vector[0][0];
	
  /**
   * @brief A = \frac{\partial f}{\partial x}
   *        1,        -halfgxdt,  -halfgydt,  -halfgzdt, (  0.5f*q1*dt,  0.5f*q2*dt )
   *        halfgxdt,  1,          halfgzdt,  -halfgydt, ( -0.5f*q0*dt,  0.5f*q3*dt )
   *        halfgydt, -halfgzdt,   1,          halfgxdt, ( -0.5f*q3*dt, -0.5f*q0*dt )
   *        halfgzdt,  halfgydt,  -halfgxdt,   1,        (  0.5f*q2*dt, -0.5f*q1*dt )
   *        0,         0,          0,          0,         1,            0 
   *        0,         0,          0,          0,         0,            1
   */
  kf->Data.A[4]  =  0.5f*kf->Data.xhatminus[1]*kf->dt;
  kf->Data.A[5]  =  0.5f*kf->Data.xhatminus[2]*kf->dt;

  kf->Data.A[10] = -0.5f*kf->Data.xhatminus[0]*kf->dt;
  kf->Data.A[11] =  0.5f*kf->Data.xhatminus[3]*kf->dt;

  kf->Data.A[16] = -0.5f*kf->Data.xhatminus[3]*kf->dt;
  kf->Data.A[17] = -0.5f*kf->Data.xhatminus[0]*kf->dt;

  kf->Data.A[22] =  0.5f*kf->Data.xhatminus[2]*kf->dt;
  kf->Data.A[23] = -0.5f*kf->Data.xhatminus[1]*kf->dt;

  /* Limit the P data */
  VAL_LIMIT(kf->Data.P[28],-10000,10000);
  VAL_LIMIT(kf->Data.P[35],-10000,10000);
}

//------------------------------------------------------------------------------
/**
  * @brief Update the measurement matrix
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @retval none
  */
static void QuaternionEKF_H_Update(KalmanFilter_Info_TypeDef *kf)
{
  /**
   * @brief H = \frac{\partial h}{\partial x}
   *  -2.f*q2,  2.f*q3, -2.f*q0, 2.f*q1, 0, 0
   *   2.f*q1,  2.f*q0,  2.f*q3, 2.f*q2, 0, 0
   *   2.f*q0, -2.f*q1, -2.f*q2, 2.f*q3, 0, 0
   */
  memset(kf->Data.H,0,kf->sizeof_float * kf->zSize * kf->xhatSize);

  kf->Data.H[0]  = -2.f*kf->Data.xhatminus[2];
  kf->Data.H[1]  =  2.f*kf->Data.xhatminus[3];
  kf->Data.H[2]  = -2.f*kf->Data.xhatminus[0];
  kf->Data.H[3]  =  2.f*kf->Data.xhatminus[1];
  kf->Data.H[6]  =  2.f*kf->Data.xhatminus[1];
  kf->Data.H[7]  =  2.f*kf->Data.xhatminus[0];
  kf->Data.H[8]  =  2.f*kf->Data.xhatminus[3];
  kf->Data.H[9]  =  2.f*kf->Data.xhatminus[2];
  kf->Data.H[12] =  2.f*kf->Data.xhatminus[0];
  kf->Data.H[13] = -2.f*kf->Data.xhatminus[1];
  kf->Data.H[14] = -2.f*kf->Data.xhatminus[2];
  kf->Data.H[15] =  2.f*kf->Data.xhatminus[3];
}
//------------------------------------------------------------------------------
/**
  * @brief Chi Square root Test
  * @param kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @retval none
  */
static bool QuaternionEKF_ChiSqrtTest(KalmanFilter_Info_TypeDef *kf)
{
  /* cache_matrix[0] = inverse(H·Pminus·HT + R)·(z(k) - h(xhatminus)) */
  kf->Mat.cache_matrix[0].numRows = kf->Mat.cache_matrix[1].numRows;
  kf->Mat.cache_matrix[0].numCols = 1;
  kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_matrix[1], &kf->Mat.cache_vector[1], &kf->Mat.cache_matrix[0]);

  /* cache_vector[0] = (z(k) - h(xhatminus)' */
  kf->Mat.cache_vector[0].numRows = 1;
  kf->Mat.cache_vector[0].numCols = kf->Mat.cache_matrix[1].numRows;
  kf->MatStatus = Matrix_Transpose(&kf->Mat.cache_matrix[1], &kf->Mat.cache_vector[0]);

  /* ChiSquare_Matrix = (z(k) - h(xhatminus)'·inverse(H·Pminus·HT + R)·(z(k) - h(xhatminus)) */
  kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_vector[0], &kf->Mat.cache_matrix[0], &kf->ChiSquareTest.ChiSquare_Matrix);

  /* rk is small,filter converged/converging */ 
  if (kf->ChiSquareTest.ChiSquare_Data[0] < 0.5f * kf->ChiSquareTest.ChiSquareTestThresholds)
  {
      kf->ChiSquareTest.result = true;
  }
  /* rk is bigger than thre but once converged */ 
  if (kf->ChiSquareTest.ChiSquare_Data[0] > kf->ChiSquareTest.ChiSquareTestThresholds && kf->ChiSquareTest.result)
  {
      if (kf->ChiSquareTest.TestFlag)
      {
          kf->ChiSquareTest.ChiSquareCnt++;
      }
      else
      {
          kf->ChiSquareTest.ChiSquareCnt = 0;
      }

      if (kf->ChiSquareTest.ChiSquareCnt > 50)
      {
          kf->ChiSquareTest.result = 0;
          kf->SkipStep5 = false; // step-5 is cov mat P updating
      }
      else
      {
          /* xhat(k) = xhat'(k) */
          /* P(k) = P'(k) */
          memcpy(kf->Data.xhat, kf->Data.xhatminus, kf->sizeof_float * kf->xhatSize);
          memcpy(kf->Data.P, kf->Data.Pminus, kf->sizeof_float * kf->xhatSize * kf->xhatSize);

          /* skip the P update */
          kf->SkipStep5 = true;
          return true;
      }
  }
  else
  {
			/* The smaller the rk , the greater the gain */
			if(kf->ChiSquareTest.ChiSquare_Data[0] > 0.1f * kf->ChiSquareTest.ChiSquareTestThresholds && kf->ChiSquareTest.result)
			{
				kf->Data.cache_vector[0][0] = (kf->ChiSquareTest.ChiSquareTestThresholds - kf->ChiSquareTest.ChiSquare_Data[0]) / (0.9f * kf->ChiSquareTest.ChiSquareTestThresholds);
			}
			else
			{
				kf->Data.cache_vector[0][0] = 1.f;
			}
			
      kf->ChiSquareTest.ChiSquareCnt = 0;
      kf->SkipStep5 = false;
  }
  return false;
}
//------------------------------------------------------------------------------
/**
  * @brief  Update the posteriori estimate matrix
  * @param  kf: pointer to a KalmanFilter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @retval none
  */
static void QuaternionEKF_xhat_Update(KalmanFilter_Info_TypeDef *kf)
{
  /* HT */
  kf->MatStatus = Matrix_Transpose(&kf->Mat.H,&kf->Mat.HT);

  /* cache_matrix[0] = H·Pminus */
  kf->Mat.cache_matrix[0].numRows = kf->Mat.H.numRows;
  kf->Mat.cache_matrix[0].numCols = kf->Mat.Pminus.numCols;
  kf->MatStatus = Matrix_Multiply(&kf->Mat.H, &kf->Mat.Pminus, &kf->Mat.cache_matrix[0]);

  /* cache_matrix[1] = H·Pminus·HT */
  kf->Mat.cache_matrix[1].numRows = kf->Mat.cache_matrix[0].numRows;
  kf->Mat.cache_matrix[1].numCols = kf->Mat.HT.numCols;
  kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_matrix[0], &kf->Mat.HT, &kf->Mat.cache_matrix[1]); 
  
  /* K_denominator = H Pminus HT + R */
  kf->Mat.K_denominator.numRows = kf->Mat.R.numRows;
  kf->Mat.K_denominator.numCols = kf->Mat.R.numCols;
  kf->MatStatus = Matrix_Add(&kf->Mat.cache_matrix[1], &kf->Mat.R, &kf->Mat.K_denominator);

  /* cache_matrix[1] = inverse(H·Pminus·HT + R) */
  kf->MatStatus = Matrix_Inverse(&kf->Mat.K_denominator, &kf->Mat.cache_matrix[1]);

  /* Calculate direction of gravity indicated by algorithm */
  kf->Mat.cache_vector[0].numRows = kf->Mat.H.numRows;
  kf->Mat.cache_vector[0].numCols = 1;
  kf->Data.cache_vector[0][0] = 2.f * (kf->Data.xhatminus[1] * kf->Data.xhatminus[3] - kf->Data.xhatminus[0] * kf->Data.xhatminus[2]);
  kf->Data.cache_vector[0][1] = 2.f * (kf->Data.xhatminus[0] * kf->Data.xhatminus[1] + kf->Data.xhatminus[2] * kf->Data.xhatminus[3]);
  kf->Data.cache_vector[0][2] = kf->Data.xhatminus[0] * kf->Data.xhatminus[0] \
                              - kf->Data.xhatminus[1] * kf->Data.xhatminus[1] \
                              - kf->Data.xhatminus[2] * kf->Data.xhatminus[2] \
                              + kf->Data.xhatminus[3] * kf->Data.xhatminus[3];

  /* the cosine of three axis orientation */
	float OrientationCosine[3];
	
  /* calculate the cosine of three axis orientation */
	for (uint8_t i = 0; i < 3; i++)
	{
		OrientationCosine[i] = acosf(fabsf(kf->Data.cache_vector[0][i]));
	}
	
  /* cache_vector[1] = z(k) - h(xhat'(k)) */
  kf->Mat.cache_vector[1].numRows = kf->Mat.z.numRows;
  kf->Mat.cache_vector[1].numCols = 1;
  kf->MatStatus = Matrix_Subtract(&kf->Mat.z, &kf->Mat.cache_vector[0], &kf->Mat.cache_vector[1]);

  /* Chi Square root Test */
  if(QuaternionEKF_ChiSqrtTest(kf)==true)
  {
    return;
  }

  /* cache_matrix[0] = Pminus·HT */
  kf->Mat.cache_matrix[0].numRows = kf->Mat.Pminus.numRows;
  kf->Mat.cache_matrix[0].numCols = kf->Mat.HT.numCols;
  kf->MatStatus = Matrix_Multiply(&kf->Mat.Pminus, &kf->Mat.HT, &kf->Mat.cache_matrix[0]);

  /* k = Pminus·HT·inverse(H·Pminus·HT + R) */
  kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_matrix[0], &kf->Mat.cache_matrix[1], &kf->Mat.K);
	
	/* The smaller the rk , the greater the gain */
	for(uint8_t i = 0; i < kf->Mat.K.numCols*kf->Mat.K.numRows; i++)
	{
		kf->Data.K[i] *= kf->Data.cache_vector[0][0];
	}

  /**
   * @brief K = \frac {P·minus·HT}{H·Pminus·HT + V·R·VT}
   *          = [  0,  1,  2,
   *               3,  4,  5,
   *               6,  7,  8,
   *               9, 10, 11, 
   *             (12, 13, 14,)
   *             (15, 16, 17,)]
   * @note  K[12..17] *=  cos(axis)/(PI/2.f)
   */
  for (uint8_t i = 4; i < 6; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
        kf->Data.K[i * 3 + j] *= OrientationCosine[i - 4] / 1.5707963f; // 1 rad
    }
  }

  /* cache_vector[0] = K(k)·(z(k) - H·xhat'(k)) */
  kf->Mat.cache_vector[0].numRows = kf->Mat.K.numRows;
  kf->Mat.cache_vector[0].numCols = 1;
  kf->MatStatus = Matrix_Multiply(&kf->Mat.K, &kf->Mat.cache_vector[1], &kf->Mat.cache_vector[0]);

  if(kf->ChiSquareTest.result)
  {
    VAL_LIMIT(kf->Data.cache_vector[0][4],-1e-2f*kf->dt,1e-2f*kf->dt);
    VAL_LIMIT(kf->Data.cache_vector[0][5],-1e-2f*kf->dt,1e-2f*kf->dt);
  }
  kf->Data.cache_vector[0][3] = 0;

  kf->MatStatus = Matrix_Add(&kf->Mat.xhatminus, &kf->Mat.cache_vector[0], &kf->Mat.xhat);
}
//------------------------------------------------------------------------------

/**
  * @brief  fast calculate the inverse square root
  * @param  x: the input variable
  * @note   see http://en.wikipedia.org/wiki/Fast_inverse_square_root
  * @retval the inverse square root of input
  */
static float Fast_InverseSqrt(float x)
{

    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;

    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//------------------------------------------------------------------------------


