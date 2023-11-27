/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : remote_control.c
  * @brief          : remote_control interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/05/23
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "remote_control.h"
#include "ramp.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
Remote_Info_Typedef remote_ctrl={
	.online_cnt = 0xFAU,
	.rc_lost = true,
};

/**
 * @brief remote control usart RxDMA MultiBuffer
 */
uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/* Private variables ---------------------------------------------------------*/
/**
 * @brief structure that contains the information of keyboard
 */
static Remote_Pressed_Typedef KeyBoard_Info;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Update the status of keyboard
  */
static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status);

/**
  * @brief  convert the remote control received message
  * @param  sbus_buf: pointer to a array that contains the information of the received message.
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;

    /* Channel 0, 1, 2, 3 */
    remote_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    remote_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    remote_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    remote_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    remote_ctrl->rc.ch[4] = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ) & 0x07ff;                 			      //!< Channel 4

    /* Switch left, right */
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right

    /* Mouse axis: X, Y, Z */
    remote_ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remote_ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis

    /* Mouse Left, Right Is Press  */
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press

    /* KeyBoard value */
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    
		/* reset the online count */
		remote_ctrl->online_cnt = 0xFAU;
		
		/* reset the lost flag */
		remote_ctrl->rc_lost = false;
}
//------------------------------------------------------------------------------

/**
  * @brief  clear the remote control data while the device offline
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void Remote_Message_Moniter(Remote_Info_Typedef  *remote_ctrl)
{
  /* Juege the device status */
  if(remote_ctrl->online_cnt <= 0x32U)
  {
    /* clear the data */
    memset(remote_ctrl,0,sizeof(Remote_Info_Typedef));

    /* reset the online count */
    remote_ctrl->online_cnt = 0xFAU;
		
    /* set the lost flag */
		remote_ctrl->rc_lost = true;
		
  }
  else if(remote_ctrl->online_cnt > 0)
  {
    /* online count decrements which reseted in received interrupt  */
    remote_ctrl->online_cnt--;
  }
}
//------------------------------------------------------------------------------

/**
  * @brief  Update the status of keyboard
  * @param  KeyInfo: pointer to a KeyBoard_Info_Typedef structure that
  *         contains the information for the keyboard.
  * @param  KeyBoard_Status: flag of the keyboard status 
  * @retval none
  */
static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status)
{ 
  /* store the keyboard status */
  KeyInfo->KEY_PRESS = KeyBoard_Status;

  /* judge the change of keyboard status */
  if(KeyInfo->last_KEY_PRESS != KeyInfo->KEY_PRESS)
  {
    /* clear the status judgement count  */
    KeyInfo->Count = 0;
  
    /* update the last keyboard status */
    KeyInfo->last_KEY_PRESS = KeyInfo->KEY_PRESS;
  }

  /* the keyboard status is key up */
  if(KEY_UP == KeyInfo->KEY_PRESS)
  {
    /* last keyboard status is key down */
    if(KEY_UP != KeyInfo->last_Status)
      KeyInfo->Count++;
    /* last keyboard status is key up */
    else
      KeyInfo->Count = 0;

    /* update the keyboard status according the judgement count */
    if(KeyInfo->Count >= KEY_SET_SHORT_TIME + 1)
    {
      KeyInfo->Status = UP;
      KeyInfo->last_Status = UP;
    }
    else if(KeyInfo->Count >= KEY_SET_SHORT_TIME)
    {
      KeyInfo->Status = RELAX;
      KeyInfo->last_Status = RELAX;
    }
  }
  /* the keyboard status is key down */
  else if(KEY_DOWN == KeyInfo->KEY_PRESS)
  {
    /* last keyboard status is key up */
    if(KEY_DOWN != KeyInfo->last_Status)
      KeyInfo->Count++;
    /* last keyboard status is key down */
    else
      KeyInfo->Count = 0;

    /* update the keyboard status according the judgement count */
    if(KeyInfo->Count >= KEY_SET_LONG_TIME)
    {
      KeyInfo->Status = DOWN;
      KeyInfo->last_Status = DOWN;
    }
    else if(KeyInfo->Count >= KEY_SET_SHORT_TIME + 1)
    {
      KeyInfo->Status = SHORT_DOWN;
      KeyInfo->last_Status = SHORT_DOWN;
    }
    else if(KeyInfo->Count >= KEY_SET_SHORT_TIME)
    {
      KeyInfo->Status = PRESS;
      KeyInfo->last_Status = PRESS;
    }
  }
}
//------------------------------------------------------------------------------

bool Key_R(void)
{
  static bool Cover_status = false;

  /* update the key status */
  Key_Status_Update(&KeyBoard_Info.R,KeyBoard_R);

  switch (KeyBoard_Info.R.Status)
  {
    case UP:
    break;

    case PRESS:
      Cover_status = !Cover_status;
    break;

    case SHORT_DOWN:
    break;

    case DOWN:
    break;

    case RELAX:
    break;

    default:break;
  }
  return Cover_status;
}

bool Key_B(void)
{
  bool Shooter_Mode_Switch = false;

  /* update the key status */
  Key_Status_Update(&KeyBoard_Info.B,KeyBoard_B);

  switch (KeyBoard_Info.B.Status)
  {
    case UP:
    break;

    case PRESS:
      Shooter_Mode_Switch = true;
    break;

    case SHORT_DOWN:
    break;

    case DOWN:
    break;

    case RELAX:
    break;

    default:break;
  }

  return Shooter_Mode_Switch;
}

bool Mouse_Pressed_Right(void)
{
  bool auto_aim = false;

  /* update the key status */
  Key_Status_Update(&KeyBoard_Info.press_r,MOUSE_PRESSED_RIGHT);

  switch (KeyBoard_Info.press_r.Status)
  {
    case UP:
    break;

    case PRESS:
      auto_aim = true;
    break;

    case SHORT_DOWN:
      auto_aim = true;
    break;

    case DOWN:
      auto_aim = true;
    break;

    case RELAX:
    
    break;

    default:break;
  }
  return auto_aim;
}

bool Mouse_Pressed_Left(void)
{
  bool fire = false;

  /* update the key status */
  Key_Status_Update(&KeyBoard_Info.press_l,MOUSE_PRESSED_LEFT);

  switch (KeyBoard_Info.press_l.Status)
  {
    case UP:
    break;

    case PRESS:
    break;

    case SHORT_DOWN:
      fire = true;
    break;

    case DOWN:
      fire = true;
    break;

    case RELAX:
    break;

    default:break;
  }
  return fire;
}

