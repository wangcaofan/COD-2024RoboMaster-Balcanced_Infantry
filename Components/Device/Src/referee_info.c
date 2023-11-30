/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : referee_info.c
  * @brief          : referee interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "referee_info.h"
#include "crc.h"
#include "string.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief Referee_RxDMA MultiBuffer
 */
uint8_t REFEREE_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];

/**
 * @brief Referee structure variable
 */
Referee_Info_TypeDef Referee_Info;

/* Private function prototypes -----------------------------------------------*/
static uint32_t bit8TObit32(uint8_t change_info[4]);
static float bit8TOfloat32(uint8_t change_info[4]);
//static uint8_t bit32TObit8(uint8_t index_need,uint32_t bit32);
static int16_t bit8TObit16(uint8_t change_info[2]);
//static uint8_t bit16TObit8(uint8_t index_need,int16_t bit16);
static void Referee_Info_Update(uint8_t *Buff,Referee_Info_TypeDef *referee);

void Referee_Frame_Update(uint8_t *Buff)
{
	Referee_Info.index = 0;
	Referee_Info.datalength = 0;
	
  while (Buff[Referee_Info.index] == 0xA5)
	{
    if(verify_CRC8_check_sum(&Buff[Referee_Info.index],FrameHeader_Length) == true)
    {
      Referee_Info.datalength = (uint16_t)(Buff[Referee_Info.index+2]<<8 | Buff[Referee_Info.index+1]) + FrameHeader_Length + CMDID_Length + CRC16_Length;
      if(verify_CRC16_check_sum(&Buff[Referee_Info.index],Referee_Info.datalength) == true)
      {
        Referee_Info_Update(Buff,&Referee_Info);
      }
    }
		Referee_Info.index += Referee_Info.datalength;
  }
}

static void Referee_Info_Update(uint8_t *Buff,Referee_Info_TypeDef *referee)
{
  switch (bit8TObit16(&Buff[referee->index + FrameHeader_Length]))
  {
#ifdef GAME_STATUS_ID
    case GAME_STATUS_ID:
      referee->game_status.game_type         = Buff[referee->index + FrameHeader_Length + CMDID_Length] & 0xF0 >> 4;
      referee->game_status.game_progress     = Buff[referee->index + FrameHeader_Length + CMDID_Length] & 0x0F;
      referee->game_status.stage_remain_time = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 1]);
    break;
#endif

#ifdef GAME_ROBOTHP_ID
    case GAME_ROBOTHP_ID:
      referee->robot_HP.red_1_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->robot_HP.red_2_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
      referee->robot_HP.red_3_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
      referee->robot_HP.red_4_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 6]);
      referee->robot_HP.red_5_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
      referee->robot_HP.red_7_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 10]);
      referee->robot_HP.red_outpost_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 12]);
      referee->robot_HP.red_base_HP    = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 14]);

      referee->robot_HP.blue_1_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 16]);
      referee->robot_HP.blue_2_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 18]);
      referee->robot_HP.blue_3_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 20]);
      referee->robot_HP.blue_4_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 22]);
      referee->robot_HP.blue_5_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 24]);
      referee->robot_HP.blue_7_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 26]);
      referee->robot_HP.blue_outpost_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 28]);
      referee->robot_HP.blue_base_HP    = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 30]);
    break;
#endif

#ifdef EVENE_DATA_ID
    case EVENE_DATA_ID:
      referee->site_event.event_type = bit8TObit32(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
    break;
#endif

#ifdef DART_REMAINING_TIME_ID
    case DART_REMAINING_TIME_ID:
      referee->dart_remaining.dart_remaining_time = Buff[referee->index + FrameHeader_Length + CMDID_Length];
    break;
#endif

#ifdef ROBOT_STATUS_ID
    case ROBOT_STATUS_ID:
      referee->robot_status.robot_id    = Buff[referee->index + FrameHeader_Length + CMDID_Length];
      referee->robot_status.robot_level = Buff[referee->index + FrameHeader_Length + CMDID_Length + 1];
      referee->robot_status.remain_HP   = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
      referee->robot_status.max_HP      = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
      referee->robot_status.shooter_id1_17mm_cooling_rate  = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 6]);
      referee->robot_status.shooter_id1_17mm_cooling_limit = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
      referee->robot_status.shooter_id1_17mm_speed_limit   = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 10]);
      referee->robot_status.shooter_id2_17mm_cooling_rate  = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 12]);
      referee->robot_status.shooter_id2_17mm_cooling_limit = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 14]);
      referee->robot_status.shooter_id2_17mm_speed_limit   = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 16]);
      referee->robot_status.shooter_id1_42mm_cooling_rate  = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 18]);
      referee->robot_status.shooter_id1_42mm_cooling_limit = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 20]);
      referee->robot_status.shooter_id1_42mm_speed_limit   = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 22]);
      referee->robot_status.chassis_power_limit        = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 24]);
      referee->robot_status.mains_power_gimbal_output  = (Buff[referee->index + FrameHeader_Length + CMDID_Length + 26] & 0x80) >> 7;
      referee->robot_status.mains_power_chassis_output = (Buff[referee->index + FrameHeader_Length + CMDID_Length + 26] & 0x40) >> 6;
      referee->robot_status.mains_power_shooter_output = (Buff[referee->index + FrameHeader_Length + CMDID_Length + 26] & 0x20) >> 5;
    break;
#endif

#ifdef REAL_POWER_HEAT_ID
    case REAL_POWER_HEAT_ID:
      referee->power_heat.chassis_volt    = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->power_heat.chassis_current = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
      referee->power_heat.chassis_power   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
      referee->power_heat.chassis_power_buffer = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
      referee->power_heat.shooter_id1_17mm_cooling_heat = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 10]);
      referee->power_heat.shooter_id2_17mm_cooling_heat = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 12]);
      referee->power_heat.shooter_id1_42mm_cooling_heat = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 14]);
    break;
#endif

#ifdef ROBOT_POSITION_ID
    case ROBOT_POSITION_ID:
      referee->robot_position.x   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->robot_position.y   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
      referee->robot_position.z   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
      referee->robot_position.yaw = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 12]);
    break;
#endif

#ifdef ROBOT_BUFF_ID
    case ROBOT_BUFF_ID:
      referee->robot_buff.power_rune_buff = Buff[referee->index + FrameHeader_Length + CMDID_Length];
    break;
#endif

#ifdef AERIAL_ENERGY_ID
    case AERIAL_ENERGY_ID:
      referee->aerial_energy.attack_time = Buff[referee->index + FrameHeader_Length + CMDID_Length];
    break;
#endif

#ifdef ROBOT_HURT_ID
    case ROBOT_HURT_ID:
      referee->robot_hurt.armor_id  = Buff[referee->index + FrameHeader_Length + CMDID_Length] & 0xF0 >> 4;
      referee->robot_hurt.hurt_type = Buff[referee->index + FrameHeader_Length + CMDID_Length] & 0x0F;
    break;
#endif

#ifdef REAL_SHOOT_DATA_ID
    case REAL_SHOOT_DATA_ID:
      referee->shoot_data.bullet_type  = Buff[referee->index + FrameHeader_Length + CMDID_Length];
      referee->shoot_data.shooter_id   = Buff[referee->index + FrameHeader_Length + CMDID_Length + 1];
      referee->shoot_data.bullet_freq  = Buff[referee->index + FrameHeader_Length + CMDID_Length + 2];
      referee->shoot_data.bullet_speed = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 3]);
    break;
#endif

#ifdef BULLET_REMAINING_ID
    case BULLET_REMAINING_ID:
      referee->bullet_remaining.bullet_remaining_num_17mm = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->bullet_remaining.bullet_remaining_num_42mm = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
      referee->bullet_remaining.coin_remaining_num        = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
    break;
#endif

#ifdef RFID_STATUS_ID
    case RFID_STATUS_ID:
      referee->RFID_Status.rfid_status = bit8TObit32(&Buff[referee->index+FrameHeader_Length+CMDID_Length]);
    break;
#endif

#ifdef DART_CLIENT_CMD_ID
    case DART_CLIENT_CMD_ID:
      referee->dart_client_cmd.dart_launch_opening_status = Buff[referee->index + FrameHeader_Length + CMDID_Length];
      referee->dart_client_cmd.dart_attack_target = Buff[referee->index + FrameHeader_Length + CMDID_Length + 1];
      referee->dart_client_cmd.target_change_time = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
      referee->dart_client_cmd.operate_launch_cmd_time = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
    break;
#endif

#ifdef GROUND_POSITION_ID
    case GROUND_POSITION_ID:
      referee->ground_robot_positio.hero_x       = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->ground_robot_positio.hero_y       = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
      referee->ground_robot_positio.engineer_x   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
      referee->ground_robot_positio.engineer_y   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 12]);
      referee->ground_robot_positio.standard_3_x = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 16]);
      referee->ground_robot_positio.standard_3_y = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 20]);
      referee->ground_robot_positio.standard_4_x = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 24]);
      referee->ground_robot_positio.standard_4_y = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 28]);
      referee->ground_robot_positio.standard_5_x = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 32]);
      referee->ground_robot_positio.standard_5_y = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 36]);
    break;
#endif

    default:break;
  }
}

/**
 * @brief  transform the bit8 to bit32
*/
static uint32_t bit8TObit32(uint8_t change_info[4])
{
	union
	{
    uint32_t bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.bit32;
}
//------------------------------------------------------------------------------

/**
 * @brief  transform the bit8 to float32
*/
static float bit8TOfloat32(uint8_t change_info[4])
{
	union
	{
    float float32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.float32;
}
//------------------------------------------------------------------------------

///**
// * @brief  transform the bit32 to bit8
//*/
//static uint8_t bit32TObit8(uint8_t index_need,uint32_t bit32)
//{
//	union
//	{
//    uint32_t  bit32;
//		uint8_t  byte[4];
//	}u32val;

//  u32val.bit32 = bit32;

//	return u32val.byte[index_need];
//}
//------------------------------------------------------------------------------

/**
 * @brief  transform the bit8 to bit16
*/
static int16_t bit8TObit16(uint8_t change_info[2])
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

	return u16val.bit16;
}
//------------------------------------------------------------------------------

///**
// * @brief  transform the bit16 to bit8
//*/
//static uint8_t bit16TObit8(uint8_t index_need,int16_t bit16)
//{
//	union
//	{
//    int16_t  bit16;
//		uint8_t  byte[2];
//	}u16val;

//  u16val.bit16 = bit16;
//	return u16val.byte[index_need];
//}
//------------------------------------------------------------------------------





