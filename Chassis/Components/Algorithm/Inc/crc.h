/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : crc.h
  * @brief          : crc check 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CRC_H
#define CRC_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief          calculate the crc8  
  */
extern uint8_t get_CRC8_check_sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
/**
  * @brief          CRC8 verify function  
  */
extern uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);
/**
  * @brief          append CRC8 to the end of data
  */
extern void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);
/**
  * @brief          calculate the crc16  
  */
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
/**
  * @brief          CRC16 verify function  
  */
extern uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
/**
  * @brief          append CRC16 to the end of data
  */
extern void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength);

#endif
