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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REFEREE_INFO_H
#define REFEREE_INFO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported defines ----------------------------------------------------------*/

#define REFEREE_RXFRAME_LENGTH  100

/**
 * @brief Referee Communication protocol format
 */
#define FrameHeader_Length    5U   /*!< the length of frame header */
#define CMDID_Length          2U   /*!< the length of CMD ID */
#define CRC16_Length          2U   /*!< the length of CRC ID */

/**
 * @brief Cmd id
 */
#define GAME_STATUS_ID                    0x0001U  /*!< game status data */
// #define GAME_RESULT_ID                    0x0002U  /*!< game result data */
// #define GAME_ROBOTHP_ID                   0x0003U  /*!< robot HP data */
// #define DART_STATUS_ID                    0x0004U  /*!< dart robot shoot status */

// #define EVENE_DATA_ID                     0x0101U  /*!< site event data */
// #define SUPPLY_ACTION_ID                  0x0102U  /*!< supply station action data */
// #define SUPPLY_BOOKING_ID                 0x0103U  /*!< booking supply data */
// #define REFEREE_WARNING_ID                0x0104U  /*!< referee warning data */
// #define DART_REMAINING_TIME_ID            0x0105U  /*!< dart shoot countdown */

#define ROBOT_STATUS_ID                   0x0201U  /*!< robot status data */
#define REAL_POWER_HEAT_ID                0x0202U  /*!< real power heat data */
// #define ROBOT_POSITION_ID                 0x0203U  /*!< robot position data */
// #define ROBOT_BUFF_ID                     0x0204U  /*!< robot buff data */
// #define AERIAL_ENERGY_ID                  0x0205U  /*!< aerial robot energy data */
// #define ROBOT_HURT_ID                     0x0206U  /*!< robot hurt data */
#define REAL_SHOOT_DATA_ID                0x0207U  /*!< real robot shoot data */
// #define BULLET_REMAINING_ID               0x0208U  /*!< bullet remain data */
#define RFID_STATUS_ID                    0x0209U  /*!< RFID status data */
// #define DART_CLIENT_CMD_ID                0x020AU  /*!< DART Client cmd data */
// #define GROUND_POSITION_ID                0x020BU  /*!< ground robot position */

#define INTERACTIVE_DATA_ID               0x0301U  /*!< robot interactive data */
#define CUSTOM_CONTROLLER_ID              0x0302U  /*!< custom controller data */
#define MINIMAP_INTERACTIVE_ID            0x0303U  /*!< mini map interactive data */
#define KEYMOUSE_INFO_ID                  0x0304U  /*!< key mouse data according the image transmission */
#define MINIMAP_RECV_ID                   0x0305U  /*!< mini map receive data */
#define MAP_SENTRY_DATA_ID                0x0307U  /*!< mini map sentry path */

/**
 * @brief Robot id
 */
#define ROBOT_RED_HERO_ID                 0x0001U
#define ROBOT_RED_ENGINEER_ID             0x0002U
#define ROBOT_RED_3_INFANTEY_ID           0x0003U
#define ROBOT_RED_4_INFANTEY_ID           0x0004U
#define ROBOT_RED_5_INFANTEY_ID           0x0005U
#define ROBOT_RED_AERIAL_INFANTEY_ID      0x0006U
#define ROBOT_RED_SENTEY_INFANTEY_ID      0x0007U
#define ROBOT_RED_DART_INFANTEY_ID        0x0008U
#define ROBOT_RED_RADAR_INFANTEY_ID       0x0009U
#define ROBOT_RED_OUTPOST_INFANTEY_ID     0x0010U
#define ROBOT_RED_BASE_INFANTEY_ID        0x0011U

#define ROBOT_BLUE_HERO_ID                0x0101U
#define ROBOT_BLUE_ENGINEER_ID            0x0102U
#define ROBOT_BLUE_3_INFANTEY_ID          0x0103U
#define ROBOT_BLUE_4_INFANTEY_ID          0x0104U
#define ROBOT_BLUE_5_INFANTEY_ID          0x0105U
#define ROBOT_BLUE_AERIAL_INFANTEY_ID     0x0106U
#define ROBOT_BLUE_SENTEY_INFANTEY_ID     0x0107U
#define ROBOT_BLUE_DART_INFANTEY_ID       0x0108U
#define ROBOT_BLUE_RADAR_INFANTEY_ID      0x0109U
#define ROBOT_BLUE_OUTPOST_INFANTEY_ID    0x0110U
#define ROBOT_BLUE_BASE_INFANTEY_ID       0x0111U

/**
 * @brief client id
 */
#define CLIENT_RED_HERO_ID                0x0101U
#define CLIENT_RED_ENGINEER_ID            0x0102U
#define CLIENT_RED_3_INFANTEY_ID          0x0103U
#define CLIENT_RED_4_INFANTEY_ID          0x0104U
#define CLIENT_RED_5_INFANTEY_ID          0x0105U
#define CLIENT_RED_AERIAL_INFANTEY_ID     0x0106U

#define CLIENT_BLUE_HERO_ID               0x0165U
#define CLIENT_BLUE_ENGINEER_ID           0x0166U
#define CLIENT_BLUE_3_INFANTEY_ID         0x0167U
#define CLIENT_BLUE_4_INFANTEY_ID         0x0168U
#define CLIENT_BLUE_5_INFANTEY_ID         0x0169U
#define CLIENT_BLUE_AERIAL_INFANTEY_ID    0x016AU

/* Exported types ------------------------------------------------------------*/
/* cancel byte alignment */
#pragma  pack(1)

/**
 * @brief typedef structure that contains the information of frame header
 */
typedef struct 
{
  uint8_t  SOF;           /*!< Data frame start byte, fixed value is 0xA5 */
  uint16_t data_length;   /*!< the length of data in the data frame */
  uint8_t  seq;           /*!< package serial number */
  uint8_t  CRC8;          /*!< Frame header CRC8 checksum */
}FrameHeader_TypeDef;

// /**
//  * @brief typedef structure that contains the information of Referee
//  */
// typedef struct
// {
//   FrameHeader_TypeDef FrameHeader;  /*!< frame header of referee received data */
//   uint16_t Cmd_id;                  /*!< cmd id  */
//   uint8_t  *Data;                   /*!< pointer to the receive data */
//   uint16_t FrameTail;               /*!< crc16 checksum , Whole package judgement */
// }Referee_Info_TypeDef;

/**
 * @brief typedef structure that contains the information of game status, id: 0x0001U
 */
typedef struct          
{
  /**
   * @brief the type of game,
            1:RMUC,
            2:RMUT,
            3:RMUA,
            4:RMUL,3v3,
            5:RMUL,1v1,
   */
	uint8_t game_type : 4;	        
	uint8_t game_progress : 4;	    /*!< the progress of game */
	uint16_t stage_remain_time;	    /*!< remain time of real progress */
  // uint64_t SyncTimeStamp;         /*!< unix time */
} ext_game_status_t;

/**
 * @brief typedef structure that contains the information of game result, id: 0x0002U
 */
typedef struct
{
  /**
   * @brief the result of game
            0:draw
            1:Red wins
            2:Blue wins
   */
 uint8_t winner;
} ext_game_result_t;

/**
 * @brief typedef structure that contains the information of robot HP data, id: 0x0003U
 */
typedef struct
{
  uint16_t red_1_robot_HP;   /*!< Red Hero HP */
  uint16_t red_2_robot_HP;   /*!< Red Engineer HP */
  uint16_t red_3_robot_HP;   /*!< Red 3 Infantry HP */
  uint16_t red_4_robot_HP;   /*!< Red 4 Infantry HP */
  uint16_t red_5_robot_HP;   /*!< Red 5 Infantry HP */
  uint16_t red_7_robot_HP;   /*!< Red Sentry HP */
  uint16_t red_outpost_HP;   /*!< Red Outpost HP */
  uint16_t red_base_HP;      /*!< Red Base HP */

  uint16_t blue_1_robot_HP;   /*!< Blue Hero HP */
  uint16_t blue_2_robot_HP;   /*!< Blue Engineer HP */
  uint16_t blue_3_robot_HP;   /*!< Blue 3 Infantry HP */
  uint16_t blue_4_robot_HP;   /*!< Blue 4 Infantry HP */
  uint16_t blue_5_robot_HP;   /*!< Blue 5 Infantry HP */
  uint16_t blue_7_robot_HP;   /*!< Blue Sentry HP */
  uint16_t blue_outpost_HP;   /*!< Blue Outpost HP */
  uint16_t blue_base_HP;      /*!< Blue Base HP */
} ext_game_robot_HP_t;

/**
 * @brief typedef structure that contains the information of dart status, id: 0x0004U
 */
typedef struct
{
  /**
   * @brief dart belong
   *        0: red
   *        1: blue
   */
  uint8_t dart_belong;
  uint16_t stage_remaining_time;
}ext_dart_status_t;

/**
 * @brief typedef structure that contains the information of site event data, id: 0x0101U
 */
typedef union
{
    /**
     * @brief the event of site
              bit 0:  status of supply station 1 recovery buff point, 1 is occupied
              bit 1:  status of supply station 2 recovery buff point, 1 is occupied
              bit 2:  status of supply station 3 recovery buff point, 1 is occupied
              bit 3:  status of energy buff attack point, 1 is occupied
              bit 4:  status of little energy buff, 1 is activating
              bit 5:  status of large energy buff, 1 is activating
              bit 6:  status of R2/B2 Ring Heights buff, 1 is occupied
              bit 7:  status of R3/B3 Trapezoidal Heights buff, 1 is occupied
              bit 8:  status of R4/B4 Trapezoidal Heights buff, 1 is occupied
              bit 9:  status of base shield, 1 is hold
              bit 10: status of outpost, 1 is survived
              bit 11-31: reserved
    */
    uint32_t event_type;

    uint32_t first_recovery_point : 1;
    uint32_t second_recovery_point : 1;
    uint32_t third_recovery_point : 1;
    uint32_t energy_buff_attack : 1;
    uint32_t little_energy_buff : 1;
    uint32_t large_energy_buff : 1;
    uint32_t ring_Heights_2_buff : 1;
    uint32_t trapezoidal_Heights_3_buff : 1;
    uint32_t trapezoidal_Heights_4_buff : 1;
    uint32_t base_shield : 1;
    uint32_t outpost_status : 1;
    uint32_t reserved : 19; 

} ext_event_data_t;

/**
 * @brief typedef structure that contains the information of supply station activation, id: 0x0102U
 */
typedef struct
{
  uint8_t supply_projectile_id;   /*!< supply station id */
  /**
   * @brief supply robot id
   *             0: robot none
   *             1: red hero
   *             2: red engineer
   *         3/4/5: red infantry
   *           101: blue hero
   *           102: blue engineer
   *   103/104/105: blue infantry
   */
  uint8_t supply_robot_id;
  /**
   * @brief supply projectile status
   *        0: closed
   *        1: preparing
   *        2: open
   */
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;  /*!< the number of supply */
} ext_supply_projectile_action_t;

/**
 * @brief typedef structure that contains the information of referee warning, id: 0x0104U
 */
typedef struct
{
  /**
   * @brief warning level
   *        1: yellow card
   *        2: red card
   *        3: negative
   */
  uint8_t level;
  uint8_t foul_robot_id; /*!< warning robot id */
} ext_referee_warning_t;

/**
 * @brief typedef structure that contains the information of dart remaining time, id: 0x0105U
 */
typedef struct
{
  uint8_t dart_remaining_time;    /*!< 15s countdown */
} ext_dart_remaining_time_t;

/**
 * @brief typedef structure that contains the information of robot status, id: 0x0201U
 */
typedef struct
{
  /**
   * @brief robot id
   *             0: robot none
   *             1: red hero
   *             2: red engineer
   *         3/4/5: red infantry
   *             6: red aerial
   *             7: red sentry
   *             8: red dart
   *             9: red radar station
   *           101: blue hero
   *           102: blue engineer
   *   103/104/105: blue infantry
   *           106: blue aerial
   *           107: blue sentry
   *           108: blue dart
   *           109: blue radar station
   */
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_HP;
  uint16_t max_HP;

  uint16_t shooter_id1_17mm_cooling_rate;
  uint16_t shooter_id1_17mm_cooling_limit;
  uint16_t shooter_id1_17mm_speed_limit;

  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
  uint16_t shooter_id2_17mm_speed_limit;

  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;

  uint16_t chassis_power_limit;

  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

/**
 * @brief typedef structure that contains the information of power heat data, id: 0x0202U
 */
typedef struct
{
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t chassis_power_buffer;
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/**
 * @brief typedef structure that contains the information of robot position data, id: 0x0203U
 */
typedef struct
{
  float x;    /*!< position x coordinate, unit: m */
  float y;    /*!< position y coordinate, unit: m */
  float z;    /*!< position z coordinate, unit: m */
  float yaw;  /*!< Position muzzle, unit: degrees */
} ext_robot_position_t;

/**
 * @brief typedef structure that contains the information of robot buff data, id: 0x0204U
 */
typedef union
{
    /**
     * @brief robot buff data
     *        bit 0: robot HP recovery status
     *        bit 1: muzzle cooling quicken
     *        bit 2: robot defense bonus
     *        bit 3: robot Attack bonus
     *        bit 4-8: reserved
     */
    uint8_t power_rune_buff;
    uint8_t HP_recovery:1;
    uint8_t cooling_quickly:1;
    uint8_t defense_bonus:1;
    uint8_t attack_bonus:1; 
    uint8_t reserved : 4; 
}ext_robot_buff_t;

/**
 * @brief typedef structure that contains the information of aerial robot energy, id: 0x0205U
 */
typedef struct
{
  /**
   * @brief remaining attack time, 30s Countdown
   */
  uint8_t attack_time;
} aerial_robot_energy_t;

/**
 * @brief typedef structure that contains the information of robot hurt, id: 0x0206U
 */
typedef struct
{
 uint8_t armor_id : 4; /*!< hurt armor id */
  /**
   * @brief hurt type
   *        0: armor hurt
   *        1: module offline
   *        2: over fire rate
   *        3: over fire heat
   *        4: over chassis power
   *        5: armor bump
   */
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/**
 * @brief typedef structure that contains the information of real shoot data, id: 0x0207U
 */
typedef struct
{
  /**
   * @brief bullet type
   *        1: 17mm
   *        2: 42mm
   */
  uint8_t bullet_type;
  /**
   * @brief shooter id
   *        1: 1 17mm shooter
   *        2: 2 17mm shooter
   *        3: 42mm shooter
   */
  uint8_t shooter_id;

  uint8_t bullet_freq; /*!< bullet frequence */
  float bullet_speed; /*!< bullet speed */
} ext_shoot_data_t;

/**
 * @brief typedef structure that contains the information of bullet remaining number, id: 0x0208U
 */
typedef struct
{
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/**
 * @brief typedef structure that contains the information of RFID status, id: 0x0209U
 */
typedef union
{
  /**
   * @brief RFID status
            bit 0:  status of RFID in base buff point
            bit 1:  status of RFID in heights buff point
            bit 2:  status of RFID in energy buff hit point
            bit 3:  status of RFID in flying slope buff point
            bit 4:  status of RFID in outpost buff point
            bit 6:  status of RFID in recovery buff point
            bit 7:  status of RFID in engineer resurrection card
            bit 8-31: reserved 
   */
 uint32_t rfid_status;
 uint32_t basebuff_status : 1;
 uint32_t heightsbuff_status : 1;
 uint32_t energyhitbuff_status : 1;
 uint32_t flyslopebuff_status : 1;
 uint32_t outpostbuff_status : 1;
 uint32_t HPrecoverybuff_status : 1;
 uint32_t rescuecardbuff_status : 1;
 uint32_t reserved : 25;
} ext_rfid_status_t;

/**
 * @brief typedef structure that contains the information of dart client data, id: 0x020AU
 */
typedef struct
{
  /**
   * @brief dart launch status
   *        1: closed
   *        2: opening or closing
   *        3: opened
   */
  uint8_t dart_launch_opening_status;
  /**
   * @brief dart attack target
   *        0: outpost
   *        1: base
   */
  uint8_t  dart_attack_target;
  uint16_t target_change_time;
  uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/**
 * @brief typedef structure that contains the information of robot position in mimi map, id: 0x020BU
 */
typedef struct
{
  float hero_x;
  float hero_y;
  float engineer_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
  float standard_5_x;
  float standard_5_y;
}ground_robot_position_t;

/**
 * @brief typedef structure that contains the information of robot mark, id: 0x020CU
 */
typedef struct
{
  uint8_t mark_hero_progress;
  uint8_t mark_engineer_progress;
  uint8_t mark_standard_3_progress;
  uint8_t mark_standard_4_progress;
  uint8_t mark_standard_5_progress;
  uint8_t mark_sentry_progress;
}radar_mark_data_t;

/**
 * @brief typedef structure that contains the information of robot interactive data, id: 0x0301U
 */
typedef struct
{
  /**
   * @brief data cmd id
   *        0x0200-0x02ff: robot interactive
   *        0x0100: client delete graph
   *        0x0101: client draw one graph
   *        0x0102: client draw two graph
   *        0x0103: client draw five graph
   *        0x0104: client draw seven graph
   *        0x0110: client draw char graph 
  */ 
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

/**
 * @brief typedef structure that contains the information of custom controller interactive, id: 0x0302U
 */
typedef struct 
{
  uint8_t data[30];
} robot_interactive_data_t;

/**
 * @brief typedef structure that contains the information of client transmit data, id: 0x0303U
 */
typedef struct
{
  /**
   * @brief target position coordinate, is 0 when transmit target robot id
   */
  float target_position_x;
  float target_position_y;
  float target_position_z;

  uint8_t commd_keyboard;
  uint16_t target_robot_ID;   /* is 0 when transmit position data */
} ext_robot_command_t;

/**
 * @brief typedef structure that contains the information of client receive data, id: 0x0305U
 */
typedef struct
{
  uint16_t target_robot_ID;
  float target_position_x;
  float target_position_y;
} ext_client_map_command_t;

/**
 * @brief typedef structure that contains the information of custom controller key mouse, id: 0x0306U
 */
typedef struct
{
 uint16_t key_value;
 uint16_t x_position:12;
 uint16_t mouse_left:4;
 uint16_t y_position:12;
 uint16_t mouse_right:4;
 uint16_t reserved;
}custom_client_data_t;

/**
 * @brief typedef structure that contains the information of sentry path, id: 0x0307U
 */
typedef struct
{
  /**
   * @brief  sentry status
   *         1: attack on target point
   *         2: defend on target point
   *         3: move to target point
   */
  uint8_t intention;
  uint16_t start_position_x;
  uint16_t start_position_y;
  int8_t delta_x[49];
  int8_t delta_y[49];
}map_sentry_data_t;

/**
 * @brief typedef structure that contains the information of Referee
 */
typedef struct 
{
  uint16_t index;
	
#ifdef GAME_STATUS_ID
  ext_game_status_t game_status;
#endif
	
#ifdef EVENE_DATA_ID
  ext_event_data_t site_event;
#endif
	
#ifdef DART_REMAINING_TIME_ID
  ext_dart_remaining_time_t dart_remaining;
#endif
	
#ifdef DART_CLIENT_CMD_ID
  ext_dart_client_cmd_t dart_client_cmd;
#endif
	
#ifdef AERIAL_ENERGY_ID
  aerial_robot_energy_t aerial_energy;
#endif

#ifdef GROUND_POSITION_ID
  ground_robot_position_t ground_robot_positio;
#endif

#ifdef GAME_ROBOTHP_ID
  ext_game_robot_HP_t robot_HP;
#endif
	
#ifdef ROBOT_STATUS_ID
  ext_game_robot_status_t robot_status;
#endif

#ifdef REAL_POWER_HEAT_ID
  ext_power_heat_data_t power_heat;
#endif

#ifdef ROBOT_POSITION_ID
  ext_robot_position_t robot_position;
#endif

#ifdef ROBOT_BUFF_ID
  ext_robot_buff_t robot_buff;
#endif

#ifdef ROBOT_HURT_ID
  ext_robot_hurt_t robot_hurt;
#endif

#ifdef REAL_SHOOT_DATA_ID
  ext_shoot_data_t shoot_data;
#endif

#ifdef BULLET_REMAINING_ID
  ext_bullet_remaining_t bullet_remaining;
#endif

#ifdef RFID_STATUS_ID
  ext_rfid_status_t RFID_Status;
#endif

}Referee_Info_TypeDef;

/* restore byte alignment */
#pragma  pack()

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief Referee_RxDMA MultiBuffer
 */
extern uint8_t REFEREE_MultiRx_Buf[2][100];

/**
 * @brief Referee structure variable
 */
extern Referee_Info_TypeDef Referee_Info;


/* Exported functions prototypes ---------------------------------------------*/
extern void Referee_Frame_Update(uint32_t *StdId, uint8_t *rxBuf);

#endif //REFEREE_INFO_H
