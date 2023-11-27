/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "vision_task.h"
#include "remote_control.h"
#include "motor.h"
#include "referee_info.h"

Control_Info_Typedef Control_Info={
		.stuck_flag = -1,
    .Measure.pit_angle = &INS_Info.pit_angle,
    .Measure.pit_gyro = &INS_Info.pit_gyro,
    .Measure.yaw_angle = &INS_Info.yaw_tolangle,
    .Measure.yaw_gyro = &INS_Info.yaw_gyro,
    .Limit_pitch.min = -24.f,
    .Limit_pitch.max = 25.f,
    .trigger_Buf = 7,
    .Measure.shootLeftSpeed = &DJI_Motor[SHOOTL].Data.velocity,
    .Measure.shootRightSpeed = &DJI_Motor[SHOOTR].Data.velocity,
    .Measure.triggerAngle = &DJI_Motor[TRIGGER].Data.angle,
    .Measure.triggerSpeed = &DJI_Motor[TRIGGER].Data.velocity,
};
/* Private variables ---------------------------------------------------------*/
PID_Info_TypeDef Gimbal_PID[2][2];
PID_Info_TypeDef Shooter_PID[2][2];

float Gimbal_PID_Param[2][2][PID_PARAMETER_NUM]={
    /* Pitch Motor */ 
	[0]={
		[0]={80.f,0.f,0.f,0.f,0.f,6000.f,},
		[1]={100.f,0.01f,0.f,0.f,5000.f,30000.f,},
	},
    /* Yaw Motor */ 
	[1]={
		[0]={80.f,0.f,0.f,0.f,0.f,6000.f,},
		[1]={90,0.12f,0.f,0,5000,30000,},
	},
};
float Shoot_PID_Param[3][PID_PARAMETER_NUM]={
    /* ShootL/R */
    [0]={13,0.1f,0,0,1000,16000,},
    /* trigger */
    [1]={200,0,0,0,0,4000,},
    [2]={13,0.24f,0,0,1000,10000,},
};

/* Private function prototypes -----------------------------------------------*/
static void Control_Task_Init(void);
static void Control_Mode_Update(Control_Info_Typedef *Control_Info);
static void Control_Target_Update(Control_Info_Typedef *Control_Info);
static void Control_Info_Update(Control_Info_Typedef *Control_Info);
static uint16_t Trigger_Speed_deliver(uint16_t cooling_rate);
static void Shooter_TargetSpeed_Update(Control_Info_Typedef *Control_Info);
static void Trigger_Stall_Handle(Control_Info_Typedef *Control_Info);

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
   TickType_t systick = 0;

  Control_Task_Init();

  osDelay(100);
  /* Infinite loop */
  for(;;)
  {
    systick = osKernelSysTick();

    Control_Mode_Update(&Control_Info);

    Control_Target_Update(&Control_Info);

    Control_Info_Update(&Control_Info);
		
		Trigger_Stall_Handle(&Control_Info);

    osDelayUntil(&systick,1);
  }
  /* USER CODE END Control_Task */
}

bool IF_Vision_Test = false;

static void Control_Task_Init(void)
{
    /* Pitch position */
    PID_Init(&Gimbal_PID[0][0],PID_POSITION,Gimbal_PID_Param[0][0]);
    /* Pitch speed */
    PID_Init(&Gimbal_PID[0][1],PID_POSITION,Gimbal_PID_Param[0][1]);
    /* Yaw position */
    PID_Init(&Gimbal_PID[1][0],PID_POSITION,Gimbal_PID_Param[1][0]);
    /* Yaw speed */
    PID_Init(&Gimbal_PID[1][1],PID_POSITION,Gimbal_PID_Param[1][1]);

    /* Shoot Left */
    PID_Init(&Shooter_PID[0][0],PID_POSITION,Shoot_PID_Param[0]);
    /* Shoot Right */
    PID_Init(&Shooter_PID[0][1],PID_POSITION,Shoot_PID_Param[0]);
    /* Trigger position */
    PID_Init(&Shooter_PID[1][0],PID_POSITION,Shoot_PID_Param[1]);
    /* Trigger speed */
    PID_Init(&Shooter_PID[1][1],PID_POSITION,Shoot_PID_Param[2]);
}

static void Control_Mode_Update(Control_Info_Typedef *Control_Info)
{
    /* Update the gimbal mode */
    if(remote_ctrl.rc.s[1] == 3 || remote_ctrl.rc.s[1] == 2)
    {
        Control_Info->gimbal_mode = GIMBAL_IMU;
    }
    else
    {
        Control_Info->gimbal_mode = GIMBAL_OFF;
    }

    /* Update the shooter mode */
    if(remote_ctrl.rc.s[1] == 2 && Control_Info->shoot_mode == SHOOTER_OFF)
    {
        Control_Info->shoot_mode = SHOOTER_SINGLE;
    }
    else if(remote_ctrl.rc.s[1] == 1 || remote_ctrl.rc.s[1] == 3)
    {
        Control_Info->shoot_mode = SHOOTER_OFF;
    }
    
    /* switch the shooter mode */
    if(Key_B() == true)
    {
        if(Control_Info->shoot_mode == SHOOTER_SINGLE)
        {
            Control_Info->shoot_mode = SHOOTER_BURSTS;
        }
        else if(Control_Info->shoot_mode == SHOOTER_BURSTS)
        {
            Control_Info->shoot_mode = SHOOTER_SINGLE;
        }
    }
    
    if(Mouse_Pressed_Right()== true)
    {
        Control_Info->gimbal_mode = GIMBAL_VISION;
        Control_Info->shoot_mode = SHOOTER_VISION;
    }
		
		if(IF_Vision_Test == true)
		{
			Control_Info->gimbal_mode = GIMBAL_VISION;
		}
}

static void Control_Target_Update(Control_Info_Typedef *Control_Info)
{
    /* Update the gimbal pitch/yaw target information */
    if(Control_Info->gimbal_mode == GIMBAL_IMU)
    {
        Control_Info->Target.pit_angle += (Control_Info->shoot_mode == SHOOTER_OFF)*remote_ctrl.rc.ch[1]*0.0001f - remote_ctrl.mouse.y * 0.002f;
        Control_Info->Target.yaw_angle -= (Control_Info->shoot_mode == SHOOTER_OFF)*remote_ctrl.rc.ch[0]*0.0003f + remote_ctrl.mouse.x * 0.0015f;
    }
    else if(Control_Info->gimbal_mode == GIMBAL_VISION && Vision_Info.IF_Aiming_Enable)
    {
        Control_Info->Target.pit_angle = Vision_Info.target_Pitch;
        Control_Info->Target.yaw_angle = Vision_Info.target_Yaw;
    }
    else
    {
        Control_Info->Target.pit_angle = *Control_Info->Measure.pit_angle;
        Control_Info->Target.yaw_angle = *Control_Info->Measure.yaw_angle;
    }
		
    VAL_LIMIT(Control_Info->Target.pit_angle,Control_Info->Limit_pitch.min,Control_Info->Limit_pitch.max);

    /* Update the shooter left/Right target information */
    if(Control_Info->shoot_mode != SHOOTER_OFF)
    {
			if(Referee_Info.robot_status.shooter_id1_17mm_speed_limit == 18)
			{
        Control_Info->Target.shootSpeed = SHOOT_SPEED_18M_S;
			}
			else if(Referee_Info.robot_status.shooter_id1_17mm_speed_limit == 30)
			{
        Control_Info->Target.shootSpeed = SHOOT_SPEED_30M_S;
			}
			else
			{
        Control_Info->Target.shootSpeed = SHOOT_SPEED_15M_S;
			}
			
			Shooter_TargetSpeed_Update(Control_Info);
    }
    else
    {
        Control_Info->Target.shootSpeed = 0;
    }

    /* Update the shooter trigger target information */
    if(Control_Info->shoot_mode == SHOOTER_SINGLE)
    {
        Control_Info->Target.triggerSpeed = 0;
        if(Control_Info->FIRE_SINGLE_ENABLE == true)
        {
            Control_Info->Target.triggerAngle += Control_Info->stuck_flag*(float)(SignBit(remote_ctrl.rc.ch[1])*360.f/Control_Info->trigger_Buf);
						Control_Info->Target.triggerAngle += Control_Info->stuck_flag*(float)(Mouse_Pressed_Left()*360.f/Control_Info->trigger_Buf);
            Control_Info->FIRE_SINGLE_ENABLE = false;
        }
    }
    else if(Control_Info->shoot_mode == SHOOTER_VISION)
    {
        Control_Info->Target.triggerSpeed = 0;
        if(Control_Info->FIRE_SINGLE_ENABLE == true)
        {
            Control_Info->Target.triggerAngle += Control_Info->stuck_flag*(float)(Vision_Info.IF_Fire_Accept*360.f/Control_Info->trigger_Buf);
            Control_Info->FIRE_SINGLE_ENABLE = false;
        }
    }
    else if(Control_Info->shoot_mode == SHOOTER_BURSTS)
    {
        if(Mouse_Pressed_Left() == true || SignBit(remote_ctrl.rc.ch[1]) == 1)
        {
            Control_Info->Target.triggerSpeed = Control_Info->stuck_flag*Trigger_Speed_deliver(Referee_Info.robot_status.shooter_id1_17mm_cooling_rate);
        }
        else
        {
            Control_Info->Target.triggerSpeed = 0;
        }
        Control_Info->Target.triggerAngle = *Control_Info->Measure.triggerAngle;
    }
    else
    {
        Control_Info->Target.triggerSpeed = 0;
        Control_Info->Target.triggerAngle = *Control_Info->Measure.triggerAngle;
    }

    if(fabsf(Shooter_PID[1][0].Err[0]) < 5.f && remote_ctrl.rc.ch[1] == 0 && Mouse_Pressed_Left() == false)
    {
        Control_Info->FIRE_SINGLE_ENABLE = true;
    }
}

static void Control_Info_Update(Control_Info_Typedef *Control_Info)
{
    Control_Info->Target.pit_gyro = f_PID_Calculate(&Gimbal_PID[0][0],Control_Info->Target.pit_angle,*Control_Info->Measure.pit_angle);
	
		if(Control_Info->gimbal_mode == GIMBAL_VISION)
		{
				Control_Info->Measure.yaw_angle = &INS_Info.yaw_angle;
		}
		else
		{
				Control_Info->Measure.yaw_angle = &INS_Info.yaw_tolangle;
		}
	
		if(fabsf(Control_Info->Target.yaw_angle - *Control_Info->Measure.yaw_angle) > 180)
		{
			Control_Info->Target.yaw_gyro = f_PID_Calculate(&Gimbal_PID[1][0],SignBit(Control_Info->Target.yaw_angle - *Control_Info->Measure.yaw_angle)*360-(Control_Info->Target.yaw_angle - *Control_Info->Measure.yaw_angle),0);
		}
		else
		{
			Control_Info->Target.yaw_gyro = f_PID_Calculate(&Gimbal_PID[1][0],Control_Info->Target.yaw_angle,*Control_Info->Measure.yaw_angle);
		}
	
    if(Control_Info->gimbal_mode != GIMBAL_OFF)
    {
        Control_Info->SendValue[Pitch] = f_PID_Calculate(&Gimbal_PID[0][1],Control_Info->Target.pit_gyro,*Control_Info->Measure.pit_gyro);
        Control_Info->SendValue[Yaw]   = f_PID_Calculate(&Gimbal_PID[1][1],Control_Info->Target.yaw_gyro,*Control_Info->Measure.yaw_gyro);
    }
    else
    {
        Control_Info->SendValue[Pitch] = 0;
        Control_Info->SendValue[Yaw] = 0;
    }

    Control_Info->SendValue[SHOOTL] = f_PID_Calculate(&Shooter_PID[0][0], Control_Info->Target.shootSpeed,*Control_Info->Measure.shootLeftSpeed);
    Control_Info->SendValue[SHOOTR] = f_PID_Calculate(&Shooter_PID[0][1],-Control_Info->Target.shootSpeed,*Control_Info->Measure.shootRightSpeed);

    if(Control_Info->shoot_mode == SHOOTER_SINGLE || Control_Info->shoot_mode == SHOOTER_VISION)
    {
        Control_Info->Target.triggerSpeed = f_PID_Calculate(&Shooter_PID[1][0],Control_Info->Target.triggerAngle,*Control_Info->Measure.triggerAngle);
    }

    Control_Info->SendValue[TRIGGER] = f_PID_Calculate(&Shooter_PID[1][1],Control_Info->Target.triggerSpeed,*Control_Info->Measure.triggerSpeed);
}

static float SpeedAdapt(float real_S , float min_S, float max_S,float up_num , float down_num)
{
	float res=0;
	static uint8_t SpeedErr_cnt=0;


  if(real_S < min_S && real_S > 8)
    SpeedErr_cnt++;
  else if(real_S >= min_S && real_S <= max_S )
		SpeedErr_cnt = 0;
	
  if(SpeedErr_cnt == 1)
  {
    SpeedErr_cnt = 0;
    res += up_num;
  }
  if(real_S > max_S)
    res -= down_num;
  return res;
}


static void Shooter_TargetSpeed_Update(Control_Info_Typedef *Control_Info)
{
	
	if(Referee_Info.shoot_data.bullet_speed != Control_Info->last_Firespeed)
	{
		if(Control_Info->Target.shootSpeed == SHOOT_SPEED_15M_S)
			Control_Info->Fire_Speed_Offset += SpeedAdapt(Referee_Info.shoot_data.bullet_speed,14.3f , 14.7f , 15 , 35);
		else if(Control_Info->Target.shootSpeed == SHOOT_SPEED_18M_S)
			Control_Info->Fire_Speed_Offset += SpeedAdapt(Referee_Info.shoot_data.bullet_speed,17.3f , 17.7f , 15 , 40);
		else if(Control_Info->Target.shootSpeed == SHOOT_SPEED_30M_S)
			Control_Info->Fire_Speed_Offset += SpeedAdapt(Referee_Info.shoot_data.bullet_speed,28.3f , 28.7f , 25 , 55);
	}
	
	Control_Info->last_Firespeed = Referee_Info.shoot_data.bullet_speed;

	Control_Info->Target.shootSpeed += Control_Info->Fire_Speed_Offset;
}

static uint16_t Trigger_Speed_deliver(uint16_t cooling_rate)
{
	uint16_t res = 0;
	
	if(cooling_rate >= 80)
	{
			if((Referee_Info.robot_status.shooter_id1_17mm_cooling_limit-Referee_Info.power_heat.shooter_id1_17mm_cooling_heat)>= 30)
				res = TRIGGER_FREQ_10_HZ;
			else 
				res = TRIGGER_FREQ_7_HZ;							
	}
	else if(cooling_rate >= 60)
	{
			if((Referee_Info.robot_status.shooter_id1_17mm_cooling_limit-Referee_Info.power_heat.shooter_id1_17mm_cooling_heat) >= 30)
				res = TRIGGER_FREQ_8_HZ;
			else
				res = TRIGGER_FREQ_5_HZ;
	}
	else if(cooling_rate >= 40)
	{
			if((Referee_Info.robot_status.shooter_id1_17mm_cooling_limit-Referee_Info.power_heat.shooter_id1_17mm_cooling_heat) >= 30)
				res = TRIGGER_FREQ_6_HZ;
			else 
				res = TRIGGER_FREQ_3_HZ;
	}
	else if(cooling_rate >= 35)
	{
			if((Referee_Info.robot_status.shooter_id1_17mm_cooling_limit-Referee_Info.power_heat.shooter_id1_17mm_cooling_heat) >= 30)
				res = TRIGGER_FREQ_6_HZ;
			else 
				res = TRIGGER_FREQ_3_HZ;
	}
	else
	{
			if((Referee_Info.robot_status.shooter_id1_17mm_cooling_limit-Referee_Info.power_heat.shooter_id1_17mm_cooling_heat) >= 20)
				res = TRIGGER_FREQ_5_HZ;
			else 
				res = 0;
	}
	
	return res;
}

static bool Judge_IF_SingeStuck(void)
{
	bool res = false;
		
	if(fabsf(Shooter_PID[1][0].Err[0]) >= 360.f/13.f)
	{
		res = true;
	}
	
	return res;
}
static bool Judge_IF_AutoBlock(void)
{
	bool res = false;
	
	if(fabsf(Shooter_PID[1][1].Err[0]) > 500 && abs(DJI_Motor[TRIGGER].Data.velocity) < 300)
	{
		res = true;
	}
	
	return res;
}

static void Trigger_Stall_Handle(Control_Info_Typedef *Control_Info)
{
	static uint16_t cnt = 0;
	
	if(Judge_IF_AutoBlock()==true || Judge_IF_SingeStuck()==true)
	{
		cnt++;
		if(cnt > 1000)
		{
			Control_Info->stuck_flag = -Control_Info->stuck_flag;
			cnt = 0;
		}
	}else{
			cnt = 0;
	}
}
