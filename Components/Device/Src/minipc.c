/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : minipc.c
  * @brief          : minipc interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "minipc.h"
#include "usbd_cdc_if.h"
#include "crc.h"

/* Private variables ---------------------------------------------------------*/
/**
 * @brief Buffer of MiniPC data to be sent
 */
uint8_t MiniPC_SendBuf[MINIPC_SENDLENGTH];

/**
 * @brief structure that contains the information for the MiniPC Receive Data.
 */
MiniPC_ReceivePacket_Typedef MiniPC_ReceivePacket = {
  .header = 0xA5,
};
/**
 * @brief structure that contains the information for the MiniPC Transmit Data.
 */
MiniPC_SendPacket_Typedef MiniPC_SendPacket = {
    .header = 0x5A,
};

/**
  * @brief  Send the MiniPC frame Information according the USB CDC
  * @param  SendPacket: pointer to MiniPC_SendPacket_Typedef structure that 
  *         contains the information for the MiniPC Transmit Data.
  * @retval none
  */
void MiniPC_SendFrameInfo(MiniPC_SendPacket_Typedef *SendPacket)
{
  /* calculate the crc16 */
  SendPacket->checksum = get_CRC16_check_sum((uint8_t *)SendPacket,MINIPC_SENDLENGTH-2,0xffff);

  /* store the MiniPC data to be sent */
  memcpy(MiniPC_SendBuf,(uint8_t *)SendPacket,MINIPC_SENDLENGTH);

  /* USB Send data */
  CDC_Transmit_FS(MiniPC_SendBuf,MINIPC_SENDLENGTH);
}
//------------------------------------------------------------------------------

/**
  * @brief  Receive the MiniPC frame Information according the USB CDC
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval none
  */
void MiniPC_RecvFrameInfo(uint8_t* Buf, const uint32_t *Len)
{
  /* Judge the crc16 */
  if(verify_CRC16_check_sum(Buf,*Len) != true)
  {
    return ;
  }

  /* judge the frame header */
  if(Buf[0] == MiniPC_ReceivePacket.header)
  {
    /* store the receive data */
    memcpy(&MiniPC_ReceivePacket,Buf,*Len);
  }
}
//------------------------------------------------------------------------------

