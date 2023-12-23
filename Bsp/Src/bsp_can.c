/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the can filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "can.h"
#include "motor.h"
#include "remote_control.h"
#include "cmsis_os.h"
/* Private variables ---------------------------------------------------------*/
/**
 * @brief the structure that contains the Information of CAN Receive.
 */
CAN_RxHeaderTypeDef USER_CAN_RxInstance;
/**
 * @brief the array that contains the Information of CAN Receive data.
 */
uint8_t USER_CAN_RxFrameData[8];

/**
 * @brief the structure that contains the Information of CAN Transmit.
 */
CAN_TxFrameTypeDef JointTxFrame[4] = {
	[Left_Anterior_Joint]={
		.hcan = &hcan2,
		.header.StdId=0x01,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	},
	[Left_Posterior_Joint]={
		.hcan = &hcan2,
		.header.StdId=0x02,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	}, 
	[Right_Anterior_Joint]={
		.hcan = &hcan2,
		.header.StdId=0x03,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	},
	[Right_Posterior_Joint]={
		.hcan = &hcan2,
		.header.StdId=0x04,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	}, 
};
CAN_TxFrameTypeDef RMD_L9025_Left_TxFrame ={
    .hcan = &hcan1,
		.header.StdId=0x141,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};
CAN_TxFrameTypeDef RMD_L9025_Right_TxFrame ={
    .hcan = &hcan1,
		.header.StdId=0x142,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};

/**
  * @brief  Configures the CAN Filter.
  * @param  None
  * @retval None
  */
void BSP_CAN_Init(void)
{
  CAN_FilterTypeDef CAN_FilterConfig = {0};

  /* Update the CAN1 filter Conifguration */
  CAN_FilterConfig.FilterActivation = ENABLE;
  CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfig.FilterIdHigh = 0x0000;
  CAN_FilterConfig.FilterIdLow = 0x0000;
  CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfig.FilterMaskIdLow = 0x0000;
  CAN_FilterConfig.FilterBank = 0;
  CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_FilterConfig.SlaveStartFilterBank = 0;

  /* configures the CAN1 filter */
  if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
  {	
      Error_Handler();
  }

  /* Start the CAN1 module. */
  HAL_CAN_Start(&hcan1);

  /* Enable CAN1 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Update the CAN2 filter Conifguration */
  CAN_FilterConfig.FilterBank = 14;
  CAN_FilterConfig.SlaveStartFilterBank = 14;

  /* configures the CAN2 filter */
  if(HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfig) != HAL_OK)
  {	
      Error_Handler();
  }

  /* Start the CAN2 module. */
  HAL_CAN_Start(&hcan2);

  /* Enable CAN2 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to transmit the Specifies message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  data: pointer to the CAN transmit data
  * @retval None
  */
//void USER_CAN_TxMessage(CAN_TypeDef *Instance,uint32_t StdId,uint8_t data[8],uint8_t length)
//{
//  static uint32_t TxMailbox = 0;
//	
//  /* Add a message to the first free Tx mailbox and activate the corresponding transmission request. */
//	if(Instance == CAN1)
//	{
//    CAN1_FrameTxInstance.StdId = StdId;
//    CAN1_FrameTxInstance.DLC = length;
//		HAL_CAN_AddTxMessage(&hcan1, &CAN1_FrameTxInstance, data, &TxMailbox);
//	}
//	else if(Instance == CAN2)
//	{
//    CAN2_FrameTxInstance.StdId = StdId;
//    CAN2_FrameTxInstance.DLC = length;
//		HAL_CAN_AddTxMessage(&hcan2, &CAN2_FrameTxInstance, data, &TxMailbox);
//	}
//}
//------------------------------------------------------------------------------
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader)
{
	
	static uint32_t TxMailbox = 0;

   //while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0 );
	//???????CAN??
	HAL_CAN_AddTxMessage(TxHeader->hcan, &TxHeader->header, TxHeader->Data, &TxMailbox);
}
/**
  * @brief  USER function to converting the CAN1 received message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  StdId: Specifies the standard identifier.
	* @param  data: array that contains the received massage.
  * @retval None
  */
uint16_t tick = 0;
static void CAN1_RxFifo0RxHandler(uint32_t *StdId,uint8_t data[8])
{

//DJI_Motor_Info_Update(StdId,data,&DJI_Motor[Right_Momentum]);
//	if(*StdId == 0x241||*StdId == 0x242){
	     RMD_Motor_Info_Update(StdId,data,&RMD_Motor[Left_Wheel]);
		   RMD_Motor_Info_Update(StdId,data,&RMD_Motor[Right_Wheel]);
//	}else{
//  	switch(data[0]&0x0F)
//	{
//		case 0x01:
//		 Damiao_Motor_Info_Update(data,&Damiao_Motor[0]);
//   if( Damiao_Motor[0].Data.Position < 0)  Damiao_Motor[0].Data.Position =  3.141593f + Damiao_Motor[0].Data.Position;
//	 else Damiao_Motor[0].Data.Position =  -3.141593f + Damiao_Motor[0].Data.Position;
//	Damiao_Motor[0].Data.Predict_Position = Damiao_Motor[0].Data.Position + Damiao_Motor[0].Data.Velocity*0.00128f;
////	  Damiao_Motor[0].Data.Predict_Velocity = (Damiao_Motor[0].Data.Position - Damiao_Motor[0].Data.Last_Position)/0.00128f;
////	  Damiao_Motor[0].Data.Last_Position =  Damiao_Motor->Data.Position;
//   	Damiao_Motor[0].Data.Angle =  Damiao_Motor[0].Data.Position * 57.2957732f;
//		break;
//	
//		case 0x02:
//			 Damiao_Motor_Info_Update(data,&Damiao_Motor[1]);
//		   Damiao_Motor[1].Data.Predict_Position= Damiao_Motor[1].Data.Position + Damiao_Motor[1].Data.Velocity*0.00128f;
////  	if( Damiao_Motor[1].Data.Position < 0){
////		    Damiao_Motor[1].Data.Position =  3.141593f + Damiao_Motor[1].Data.Position;
////		}else{
////		    Damiao_Motor[1].Data.Position =  -3.141593f + Damiao_Motor[1].Data.Position;
////		
////		}
////		   Damiao_Motor[1].Data.Angle =  Damiao_Motor[1].Data.Position * 57.2957732f;
//		break;
//		
//		case 0x03:
//			 Damiao_Motor_Info_Update(data,&Damiao_Motor[2]);
//		break;
//		
//		case 0x04:
//			 Damiao_Motor_Info_Update(data,&Damiao_Motor[3]);
//		break;
//		default:
//			break;
//	 }
// }
  

}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to converting the CAN2 received message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  StdId: Specifies the standard identifier.
	* @param  data: array that contains the received massage.
  * @retval None
  */
static void CAN2_RxFifo0RxHandler(uint32_t *StdId,uint8_t data[8])
{

			switch(data[0]&0x0F){
			 case 0x01:
				 Damiao_Motor_Info_Update(data,&Damiao_Motor[0]);
         Damiao_Motor[0].Data.Position =  3.141593f + Damiao_Motor[0].Data.Position;
				 Damiao_Motor[0].Data.Angle =  Damiao_Motor[0].Data.Position * 57.2957732f;
			 break;
       case 0x02:
				 Damiao_Motor_Info_Update(data,&Damiao_Motor[1]);
			 break;
			 case 0x03:
				Damiao_Motor_Info_Update(data,&Damiao_Motor[2]);
			 break;
       case 0x04:
				Damiao_Motor_Info_Update(data,&Damiao_Motor[3]);
			  Damiao_Motor[3].Data.Position =  3.141593f + Damiao_Motor[3].Data.Position;
			  Damiao_Motor[3].Data.Angle =  Damiao_Motor[3].Data.Position * 57.2957732f;
			 break;
			break;
			 default:
			 break;
			}

//	DJI_Motor_Info_Update(StdId,data,&DJI_Motor[Yaw]);
//#if (!REMOTE_FRAME_USART_CAN)
//	Remote_Info_Update(StdId,data,&remote_ctrl);
//#endif
}
//------------------------------------------------------------------------------

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get an CAN frame from the Rx FIFO zone into the message RAM. */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &USER_CAN_RxInstance, USER_CAN_RxFrameData);

  /* judge the instance of receive frame data */
  if(hcan->Instance == CAN1)
  {
    CAN1_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId,USER_CAN_RxFrameData);
  }
  else if(hcan->Instance == CAN2)
  {
    CAN2_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId,USER_CAN_RxFrameData);
  }
}
//------------------------------------------------------------------------------
