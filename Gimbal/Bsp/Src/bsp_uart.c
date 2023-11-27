/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
#include "usart.h"
#include "remote_control.h"

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t *, uint32_t);


/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void)
{
  /* Starts the remote control multi_buffer DMA Transfer with interrupt enabled. */
	USART_RxDMA_MultiBufferStart(&huart3,(uint32_t *)&(huart3.Instance->DR),(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);
}
//------------------------------------------------------------------------------

/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart       pointer to a UART_HandleTypeDef structure that contains
  *                     the configuration information for the specified USART Stream.  
  * @param  SrcAddress pointer to The source memory Buffer address
  * @param  DstAddress pointer to The destination memory Buffer address
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval none
  */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{	
  /* configuare the huart Reception Type TOIDLE */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  /* configuare the huart Receicve Size */
	huart->RxXferSize = SBUS_RX_BUF_NUM;
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Disable DMA */
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;

  /* Configure the destination memory Buffer address */
  huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  huart->hdmarx->Instance->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER USART3 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
  /* Current memory buffer used is Memory 0 */
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
  {
			//Disable DMA 
			__HAL_DMA_DISABLE(huart->hdmarx);

			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
      /* reset the receive count */
      __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

      if(Size == RC_FRAME_LENGTH)
      {
        SBUS_TO_RC(SBUS_MultiRx_Buf[0],&remote_ctrl);
      }
  }
  /* Current memory buffer used is Memory 1 */
  else
  {
			//Disable DMA 
			__HAL_DMA_DISABLE(huart->hdmarx);

			huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
		
      /* reset the receive count */
      __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

      if(Size == RC_FRAME_LENGTH)
      {
        SBUS_TO_RC(SBUS_MultiRx_Buf[1],&remote_ctrl);
      }
  }
}
//------------------------------------------------------------------------------

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{

	if(huart->Instance == USART3)
	{
		USER_USART3_RxHandler(huart,Size);
	}

  /* reset the Reception Type */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------

