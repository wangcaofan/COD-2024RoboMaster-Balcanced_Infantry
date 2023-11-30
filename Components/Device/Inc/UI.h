/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UI.h
  * @brief          : UI functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UI_H
#define UI_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

#define JUDGE_FRAME_HEADER 0xA5

/* Exported types ------------------------------------------------------------*/

typedef enum
{
	NONE_delete    = 0,
	GRAPHIC_delete = 1,
	ALL_delete     = 2
}delete_Graphic_Operate;

typedef enum
{
	NONE   = 0,
	ADD    = 1,
	MODIFY = 2,
	DELETE = 3,
}Graphic_Operate;

typedef enum
{
	LINE      = 0,
	RECTANGLE = 1,
	CIRCLE    = 2,
	OVAL      = 3,
	ARC       = 4,
	FLOAT     = 5,
	INT       = 6,
	CHAR      = 7 
}Graphic_Type;

typedef enum
{
	RED_BLUE  = 0,	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,
	PINK      = 5,
	CYAN_BLUE = 6,
	BLACK     = 7,
	WHITE     = 8,
}Graphic_Color;

enum
{
	INTERACT_ID_delete_graphic 			= 0x0100,
	INTERACT_ID_draw_one_graphic 		= 0x0101,
	INTERACT_ID_draw_two_graphic 		= 0x0102,
	INTERACT_ID_draw_five_graphic 		= 0x0103,
	INTERACT_ID_draw_seven_graphic 		= 0x0104,
	INTERACT_ID_draw_char_graphic 		= 0x0110,
	INTERACT_ID_bigbome_num				= 0x02ff,
};

/* cancel byte alignment */
#pragma  pack(1)

typedef struct
{
  uint8_t graphic_name[3];  
  uint32_t operate_tpye:3;  
  uint32_t graphic_tpye:3;  
  uint32_t layer:4;  
  uint32_t color:4;  
  uint32_t start_angle:9; 
  uint32_t end_angle:9; 
  uint32_t width:10;  
  uint32_t start_x:11;  
  uint32_t start_y:11;  
  uint32_t radius:10;  
  uint32_t end_x:11;  
  uint32_t end_y:11;  
}graphic_data_struct_t;

typedef struct
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
	float number;       
} Float_data_struct_t;

typedef struct
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  int number;       
} Int_data_struct_t;

typedef struct
{
	uint8_t operate_type; 
	uint8_t layer;
}ext_client_custom_graphic_delete_t;

typedef struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} xFrameHeader;

typedef struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 

typedef struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	ext_client_custom_graphic_delete_t clientData;		
	uint16_t	FrameTail;								
}ext_deleteLayer_data_t;

typedef struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_string_t;

typedef struct
{
	xFrameHeader txFrameHeader;
	uint16_t  CmdID;
	ext_student_interactive_header_data_t   dataFrameHeader;
	ext_client_string_t clientData;
	uint16_t	FrameTail;
}ext_charstring_data_t;

typedef  struct
{
	xFrameHeader txFrameHeader;
	uint16_t  CmdID;
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData;
	uint16_t	FrameTail;
}ext_graphic_one_data_t;
typedef  struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_graphic_two_data_t;
typedef  struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[5];		
	uint16_t	FrameTail;								
}ext_graphic_five_data_t;
typedef  struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_graphic_seven_data_t;

typedef  struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Float_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_float_two_data_t;
typedef  struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Float_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_float_seven_data_t;

typedef  struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Int_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_int_two_data_t;
typedef  struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Int_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_int_seven_data_t;

typedef enum
{
	/* Std */
	LEN_FRAME_HEAD 	                 = 5,
	LEN_CMD_ID 		                   = 2,
	LEN_FRAME_TAIL 	                 = 2,
	/* Ext */  
	LEN_game_state       				=  11,
	LEN_game_result       				=  1,
	LEN_game_robot_survivors       		=  32,
	LED_game_missile_state      =3  ,
	LED_game_buff               =11 ,
	
	LEN_event_data  					=  4,
	LEN_supply_projectile_action        =  4,
	LEN_supply_warm        =2,
	LEN_missile_shoot_time =1  ,
	
	LEN_game_robot_state    			= 27,	//0x0201
	LEN_power_heat_data   				= 16,	//0x0202
	LEN_game_robot_pos        			= 16,	//0x0203
	LEN_buff_musk        				=  1,	//0x0204
	LEN_aerial_robot_energy        		=  1,	//0x0205
	LEN_robot_hurt        				=  1,	//0x0206
	LEN_shoot_data       				=  7,	//0x0207
	LEN_bullet_remaining          = 6,//剩余发射数
  
	LEN_rfid_status					         = 4,
	LEN_dart_client_directive        = 12,//0x020A
	// 0x030x
	//LEN_robot_interactive_header_data      = n,
	//LEN_controller_interactive_header_data = n,
	LEN_map_interactive_headerdata           = 15,
	LEN_keyboard_information                 = 12,//0x0304
}JudgeDataLength;


/* restore byte alignment */
#pragma  pack()


typedef enum{
  SHOOTER_OFF,
  SHOOTER_SINGLE,
  SHOOTER_BURSTS,
  SHOOTER_VISION,
  SHOOTER_MODE_NUM,
}Shooter_Mode_e;

typedef struct
{
	bool Fire;
	bool Auto;
	Shooter_Mode_e Shooter_Mode;
	bool Cover;
}UI_Char_Typedef;

extern UI_Char_Typedef UI_Char;

extern void Start_UI_task(void);

#endif // UI_H


