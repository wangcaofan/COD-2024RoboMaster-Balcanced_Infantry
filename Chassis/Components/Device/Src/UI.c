/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UI.c
  * @brief          : UI functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "UI.h"
#include "Control_Task.h"
#include "referee_info.h"
#include "INS_Task.h"
#include "usart.h"
#include "stm32f4xx.h"
#include "crc.h"
#include "string.h"
#include "math.h"
#include "arm_math.h"
#include "config.h"

#define Robot_Receiver_ID  (Referee_Info.robot_status.robot_id | 0x100)

uint8_t update_balance_flag = 0;

UI_Char_Typedef UI_Char;

ext_charstring_data_t tx_client_char;
ext_graphic_five_data_t tx_client_line;

uint8_t state_first_graphic = 0 ;
uint8_t CliendTxBuffer[200];
char first_line[30]   = {"FRIE"};

char second1_line[30]  = {"OFF"};
char second2_line[30]  = {"SINGLE"};
char second3_line[30]  = {"BURSTS"};
char second4_line[30]  = {"VISION"};

char third1_line[30] = {"WEAK"};
char third2_line[30] = {"STOP"};
char third3_line[30] = {"BALANCE"};
char third4_line[30] = {"FRONT"};
char third5_line[30] = {"SIDE"};
char third6_line[30] = {"SPIN"};
char third7_line[30] = {"SLIP"};

char fourth_line[30] = {"AUTO"};
char fifth_line[30]  = {"COVER"};
char empty_line[30]   = {"                              "};

void Char_Graphic(ext_client_string_t* graphic,
                    const char* name,
                    uint32_t operate_tpye,
                    uint32_t layer,
                    uint32_t color,
                    uint32_t size,
                    uint32_t length,
                    uint32_t width,
                    uint32_t start_x,
                    uint32_t start_y,
                    const char *character)
{
	graphic_data_struct_t *data_struct = &graphic->grapic_data_struct;
	for(char i=0;i<3;i++)
		data_struct->graphic_name[i] = name[i];	
	data_struct->operate_tpye = operate_tpye; 
	data_struct->graphic_tpye = CHAR;         
	data_struct->layer = layer;
	data_struct->color = color;
	data_struct->start_angle = size;
	data_struct->end_angle = length;	
	data_struct->width = width;
	data_struct->start_x = start_x;
	data_struct->start_y = start_y;	
	
	data_struct->radius = 0;
	data_struct->end_x = 0;
	data_struct->end_y = 0;
	memcpy(graphic->data,empty_line,19);
    memcpy(graphic->data,character,length);
}
void Figure_Graphic(graphic_data_struct_t* graphic,
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,
									uint32_t layer,
									uint32_t color,
									uint32_t start_angle,
									uint32_t end_angle,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									uint32_t radius,
									uint32_t end_x,
									uint32_t end_y)							
{
	for(char i=0;i<3;i++)
	graphic->graphic_name[i] = name[i];	
	graphic->operate_tpye = operate_tpye; 
	graphic->graphic_tpye = graphic_tpye;     
	graphic->layer        = layer;
	graphic->color        = color;
	graphic->start_angle  = start_angle;
	graphic->end_angle    = end_angle;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->radius = radius;
	graphic->end_x  = end_x;
	graphic->end_y  = end_y;
}

static void Draw_char()
{
	if(state_first_graphic == 0)
	{
		if(UI_Char.Fire == true)
			Char_Graphic(&tx_client_char.clientData,"CL1",ADD,0,GREEN,15,strlen(first_line),2,(30),(1080*16/20),first_line);//x1920/18
		else
			Char_Graphic(&tx_client_char.clientData,"CL1",ADD,0,YELLOW,15,strlen(first_line),2,(30),(1080*16/20),first_line);//x1920/18
		state_first_graphic = 1;
	}
	else if(state_first_graphic == 1)
	{
		if(UI_Char.Shooter_Mode == SHOOTER_OFF)
		{
			Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,YELLOW,15,strlen(second1_line),2,(30),(1080*15/20),second1_line);//x1920/18
		}
		else if(UI_Char.Shooter_Mode == SHOOTER_SINGLE)
		{
			Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,GREEN,15,strlen(second2_line),2,(30),(1080*15/20),second2_line);//x1920/18
		}
		else if(UI_Char.Shooter_Mode == SHOOTER_BURSTS)
		{
			Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,GREEN,15,strlen(second3_line),2,(30),(1080*15/20),second3_line);//x1920/18
		}
		else if(UI_Char.Shooter_Mode == SHOOTER_VISION)
		{
			Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,GREEN,15,strlen(second4_line),2,(30),(1080*15/20),second4_line);//x1920/18
		}
		state_first_graphic = 2;
	}
	else if(state_first_graphic == 2)
	{
		if(Control_Info.mode == CHASSIS_WEAK)
		{
			Char_Graphic(&tx_client_char.clientData,"CL3",ADD,0,YELLOW,15,strlen(third1_line),2,(30),(1080*14/20),third1_line);//x1920/18
		}
		else if(Control_Info.mode == CHASSIS_STOP)
		{
			Char_Graphic(&tx_client_char.clientData,"CL3",ADD,0,GREEN,15,strlen(third2_line),2,(30),(1080*14/20),third2_line);//x1920/18
		}
		else if(Control_Info.mode == CHASSIS_BALANCE)
		{
			Char_Graphic(&tx_client_char.clientData,"CL3",ADD,0,GREEN,15,strlen(third3_line),2,(30),(1080*14/20),third3_line);//x1920/18
		}
		else if(Control_Info.mode == CHASSIS_FRONT)
		{
			Char_Graphic(&tx_client_char.clientData,"CL3",ADD,0,GREEN,15,strlen(third4_line),2,(30),(1080*14/20),third4_line);//x1920/18
		}
		else if(Control_Info.mode == CHASSIS_SIDE)
		{
			Char_Graphic(&tx_client_char.clientData,"CL3",ADD,0,GREEN,15,strlen(third5_line),2,(30),(1080*14/20),third5_line);//x1920/18
		}
		else if(Control_Info.mode == CHASSIS_SPIN)
		{
			Char_Graphic(&tx_client_char.clientData,"CL3",ADD,0,GREEN,15,strlen(third6_line),2,(30),(1080*14/20),third6_line);//x1920/18
		}
		else if(Control_Info.mode == CHASSIS_SLIP)
		{
			Char_Graphic(&tx_client_char.clientData,"CL3",ADD,0,GREEN,15,strlen(third7_line),2,(30),(1080*14/20),third7_line);//x1920/18
		}
		state_first_graphic = 3;
	}
	else if(state_first_graphic == 3)
	{
		if(UI_Char.Auto == true)
		{
		Char_Graphic(&tx_client_char.clientData,"CL4",ADD,0,GREEN,15,strlen(fourth_line),2,(30),(1080*13/20),fourth_line);
		}
		else
		{
		Char_Graphic(&tx_client_char.clientData,"CL4",ADD,0,YELLOW,15,strlen(fourth_line),2,(30),(1080*13/20),fourth_line);
		}
		state_first_graphic = 4;
	}
	else if(state_first_graphic == 4)
	{
		if(UI_Char.Cover == true)
		{
		Char_Graphic(&tx_client_char.clientData,"CL5",ADD,0,GREEN,15,strlen(fifth_line),2,(30),(1080*12/20),fifth_line);//x1920/18
		}
		else
		{
		Char_Graphic(&tx_client_char.clientData,"CL5",ADD,0,YELLOW,15,strlen(fifth_line),2,(30),(1080*12/20),fifth_line);//x1920/18
		}
		state_first_graphic = 5;
	}
}

void Client_Char_Init(void)
{
	if(state_first_graphic >= 5)
	{
		state_first_graphic = 0;
	}
    tx_client_char.txFrameHeader.SOF = 0xA5;
    tx_client_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
    tx_client_char.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer,&tx_client_char.txFrameHeader,sizeof(xFrameHeader));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));

    tx_client_char.CmdID = 0x0301;
    
    tx_client_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
    tx_client_char.dataFrameHeader.send_ID     = Referee_Info.robot_status.robot_id;
    tx_client_char.dataFrameHeader.receiver_ID = Robot_Receiver_ID;
    
    Draw_char();
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_char.CmdID, LEN_CMD_ID+tx_client_char.txFrameHeader.DataLength);
    
    append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_char));

    HAL_UART_Transmit_DMA(&huart1,CliendTxBuffer,sizeof(tx_client_char));
}

  int16_t LINE_OFFSET[2] = {0,};

//static void Draw_line(void)
//{
//	
//	Figure_Graphic(&tx_client_line.clientData[0],"AL1",ADD,LINE,3,YELLOW,0,0,3,540,0, 0,640,100);
//	Figure_Graphic(&tx_client_line.clientData[1],"AL2",ADD,LINE,3,YELLOW,0,0,3,1380,0, 0,1280,100);

//    if(fabsf(Control_Info.Measure.yaw_angle - Control_Info.Target.yaw_angle[1]) < fabsf(Control_Info.Measure.yaw_angle - Control_Info.Target.yaw_angle[0]))
//    {
//        Figure_Graphic(&tx_client_line.clientData[2],"AL3",update_balance_flag,LINE,2,YELLOW,0,0,3,1770,(1080*16/20), 0,1770,(1080*14/20));
//    }
//    else
//    {
//        Figure_Graphic(&tx_client_line.clientData[2],"AL3",update_balance_flag,LINE,2,YELLOW,0,0,3,1716,(1080*15/20), 0,1824,(1080*15/20));
//    }

//    LINE_OFFSET[0] = Control_Info.Chassis_Direction * arm_sin_f32(INS_Info.angle[IMU_ANGLE_INDEX_PITCH]) * 100;
//    LINE_OFFSET[1] = Control_Info.Chassis_Direction * arm_cos_f32(INS_Info.angle[IMU_ANGLE_INDEX_PITCH]) * 100;
//    
//    if(LINE_OFFSET[0] > 100) LINE_OFFSET[0] = 100;
//    else if(LINE_OFFSET[0] < -100) LINE_OFFSET[0] = -100;
//		
//    if(LINE_OFFSET[1] > 100) LINE_OFFSET[1] = 100;
//    else if(LINE_OFFSET[1] < -100) LINE_OFFSET[1] = -100;


//    Figure_Graphic(&tx_client_line.clientData[3],"AL4",update_balance_flag,LINE,2,YELLOW,0,0,3,1770,(1080*12/20), 0,(1770 - LINE_OFFSET[0]),(1080*12/20)+LINE_OFFSET[1]);
//}

//void Client_Graph_Init(void)
//{
//	tx_client_line.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
//	tx_client_line.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*5;
//	tx_client_line.txFrameHeader.Seq = 0;
//	memcpy(CliendTxBuffer,&tx_client_line.txFrameHeader,sizeof(xFrameHeader));
//	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));

//	tx_client_line.CmdID = 0x301;

//	tx_client_line.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
//	tx_client_line.dataFrameHeader.send_ID     = Referee_Info.robot_status.robot_id;
//	tx_client_line.dataFrameHeader.receiver_ID = Robot_Receiver_ID;

//	Draw_line();
//	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_line.CmdID, LEN_CMD_ID+tx_client_line.txFrameHeader.DataLength);

//	append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_line));
//		
//    HAL_UART_Transmit_DMA(&huart1,CliendTxBuffer,sizeof(tx_client_line));
//}


void Start_UI_task(void)
{	
  static uint8_t i = 0, J = 0;

  if(J == 200)
  {
    update_balance_flag = ADD;
    J = 0;
  }

  switch(i)
  {
    case 1:
        Client_Char_Init();
    break;
    
    case 2:
        //Client_Graph_Init();
        update_balance_flag = MODIFY;
		break;

    default:
        i = 0;
    break;
  }

  i++;
  J++;
}

