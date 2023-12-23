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
#include "remote_control.h"
#include "ramp.h"
#include "pid.h"
#include "motor.h"
#include "arm_math.h"
#include "referee_info.h"
#include "VMC_Task.h"
#include "bsp_uart.h"
#include "math.h"
#define printf(title, fmt, args...) usart_printf("{"#title"}"fmt"\n", ##args)
Control_Info_Typedef Left_Control_Info={
    .Control_USAGE = Left_Control,
	  .Moment={
		   .T =0,
	     .Tp=0,	
		 },
};
Control_Info_Typedef Right_Control_Info={
	  .Moment={
		   .T =0,
	     .Tp=0,	
		 },
    .Control_USAGE = Right_Control,
};
VMC_Info_Typedef Left_VMC_Info={
   .L1 = 0.15f,
   .L4 = 0.15f,
	 .L2 = 0.27f,
   .L3 = 0.27f,
	 .L5 = 0.11f,
   .VMC_USAGE = Left_Control,
	 .Phi1 = &Damiao_Motor[Left_Anterior_Joint].Data.Position,
   .Phi4 = &Damiao_Motor[Left_Posterior_Joint].Data.Position,
	 .Target_L0 = 0.f,
};
VMC_Info_Typedef Right_VMC_Info={
   .L1 = 0.15f,
   .L4 = 0.15f,
	 .L2 = 0.27f,
   .L3 = 0.27f,
	 .L5 = 0.11f,
   .VMC_USAGE = Right_Control,
	 .Phi1 = &Damiao_Motor[Right_Posterior_Joint].Data.Position,
   .Phi4 = &Damiao_Motor[Right_Anterior_Joint].Data.Position,
	 .Target_L0 = 0.f,
}; 

#define fp32 float
PID_Info_TypeDef PID_Leg_length_thrust[2];
PID_Info_TypeDef PID_Left_Turn_Angle;
PID_Info_TypeDef PID_Left_Turn_Velocity;
PID_Info_TypeDef PID_Right_Turn_Angle;
PID_Info_TypeDef PID_Right_Turn_Velocity;
PID_Info_TypeDef PID_Leg_Coordinate;
PID_Info_TypeDef PID_Roll;
Tracking_Differentiator_Info_TypeDef  Target_Velocity_TD;
fp32 a11[6] = {0,-144.762559,177.829939,-97.123843,1.334080};
fp32 a12[6] = {0,0.888953,0.903232,-8.040658,0.276474};
fp32 a13[6] = {0,-68.794432,69.495171,-24.383997,-0.028607};
fp32 a14[6] = {0,-64.976308,66.387949,-24.704114,-0.083258};
fp32 a15[6] = {0,-35.645946,75.173798,-52.106537,15.938898};
fp32 a16[6] = {0,4.534287,-1.952995,-1.121373,1.144571};
fp32 a21[6] = {0,822.763330,-694.843468,152.221112,20.260597};
fp32 a22[6] = {0,92.568564,-88.420097,26.504413,1.239193};
fp32 a23[6] = {0,-141.118103,212.091520,-117.849533,28.171964};
fp32 a24[6] = {0,-124.256664,191.087195,-108.601758,27.060084};
fp32 a25[6] = {0,1920.350421,-1970.664259,710.340171,-21.663822};
fp32 a26[6] = {0,107.620660,-119.449610,48.188830,-4.797264};
static float PID_Leg_length_thrust_param[6]={500.f,0.1f,10000.f,0,30,100};
static float PID_Turn_Angle_param0[6]={50.f,0.0f,5.f,0,0,100};
static float PID_Turn_Velocity_param0[6]={-0.05f,0.0f,0.0f,0,0,1.4f};
static float PID_Turn_Angle_param1[6]={50.f,0.0f,5.f,0,0,100};
static float PID_Turn_Velocity_param1[6]={-0.05f,0.0f,0.0f,0,0,1.4f};
static float PID_Leg_Coordinate_param[6]={20.f,0.0f,2.0f,0,0,10};
static float PID_Roll_param[6]={10.f,0.0f,0.0f,0,0,100};

static void Control_Mode_Update(Control_Info_Typedef *Control_Info);
static void VMC_Calculate(VMC_Info_Typedef *VMC_Info);
static void VMC_Calculate1(VMC_Info_Typedef *VMC_Info);
static void Control_Measure_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info);
static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info);
static void Control_LQR_K_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info);
static void Control_LQR_U_Calculate(Control_Info_Typedef *Control_Info);
static void VMC_F_Tp_To_Joint_Calculate(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info);
static void VMC_F_Tp_To_Joint_Calculate1(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info);
 float qiankui1,qiankui0,F_Roll= 0;
bool flag1=0,flag2=0;
/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
//int16_t tick;
float Measure_Chassis_Velocity = 0;
float Target_Chassis_Velocity =0;
float Last_Target_Chassis_Velocity= 0;
float ACC = 0;
float Target_Chassis_Position =0;
float Measure_Chassis_Position=0;
float Target_Angle=0;
int16_t Balance_time = 0;
float Leg_Coordinate_Tp,Left_Turn_T,Right_Turn_T;
bool Init = 0;
  float Target;
bool init1,init0=0;

void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  TickType_t systick = 0;
	PID_Init(&PID_Leg_length_thrust[0],PID_POSITION,PID_Leg_length_thrust_param);
	PID_Init(&PID_Leg_length_thrust[1],PID_POSITION,PID_Leg_length_thrust_param);
	PID_Init(&PID_Left_Turn_Angle,PID_POSITION,PID_Turn_Angle_param0);
	PID_Init(&PID_Left_Turn_Velocity,PID_POSITION,PID_Turn_Velocity_param0);
	PID_Init(&PID_Right_Turn_Angle,PID_POSITION,PID_Turn_Angle_param1);
	PID_Init(&PID_Right_Turn_Velocity,PID_POSITION,PID_Turn_Velocity_param1);
	PID_Init(&PID_Roll,PID_POSITION,PID_Roll_param);
 /* Infinite loop */
	for(;;)
  {
    systick = osKernelSysTick();
   if(fabsf(INS_Info.pit_angle)>=13.f&&remote_ctrl.rc.s[1]!=1){
	  Left_Control_Info.mode =  CHASSIS_WEAK;
	  Right_Control_Info.mode  =  CHASSIS_WEAK;
		 Balance_time = 0;
		  Target_Angle = INS_Info.yaw_tolangle;	
     		 
	 }else if(fabsf(INS_Info.pit_angle)<13.f&&remote_ctrl.rc.s[1]==1){
	     
		  if( Left_Control_Info.mode == CHASSIS_WEAK) Balance_time++;
	     if(Balance_time>500){
			 Left_Control_Info.mode =  CHASSIS_BALANCE;
	     Right_Control_Info.mode  =  CHASSIS_BALANCE;
			 Balance_time = 0;  
			 }
 }
		VMC_Calculate(&Left_VMC_Info);
    VMC_Calculate1(&Right_VMC_Info);
    Control_LQR_K_Update(&Left_Control_Info,&Left_VMC_Info);
    Control_LQR_K_Update(&Right_Control_Info,&Right_VMC_Info);
		
	  

   
	// f_Ramp_Calc(Target_Chassis_Velocity,remote_ctrl.rc.ch[3]*0.00076f,0.001);

	if(remote_ctrl.rc.ch[3] != 0){
	Target =  remote_ctrl.rc.ch[3]*0.0025f;
	}else{
	 Target = 0;
	}
	VAL_LIMIT(Target,-1.5f,1.5f);
	ACC = ( Target - Target_Chassis_Velocity)/0.002f;
	if(fabsf(ACC)<1.f){
	Target_Chassis_Velocity  =  Target;
	}else{
	  Target_Chassis_Velocity += sign(ACC)*0.002f;
	}
	Last_Target_Chassis_Velocity = Target_Chassis_Velocity;
	if(remote_ctrl.rc.s[1]==1){
	Target_Chassis_Position += Target_Chassis_Velocity*0.002f;
  }else  Target_Chassis_Position = 0;
if(Target_Chassis_Velocity != 0) flag1 =1;

Measure_Chassis_Velocity =((-RMD_Motor[0].Data.Rad_Velocity + RMD_Motor[1].Data.Rad_Velocity)/2.f)*0.08f ;//+INS_Info.gyro[2]-((Left_VMC_Info.Phi0_dot-Right_VMC_Info.Phi0_dot)/2))*0.08f;		

	Control_Measure_Update(&Left_Control_Info,&Left_VMC_Info);
 Control_Measure_Update(&Right_Control_Info,&Right_VMC_Info);

 if(Target_Chassis_Velocity==0 &&  flag1==1 && fabsf(Left_Control_Info.Measure.Theta)<=0.04f){
  Target_Chassis_Position = 0;
  Left_Control_Info.Measure.Chassis_Position = 0;
  Right_Control_Info.Measure.Chassis_Position = 0;
	flag1 = 0 ;
}	
 Control_LQR_X_Update(&Left_Control_Info,&Left_VMC_Info);
 Control_LQR_X_Update(&Right_Control_Info,&Right_VMC_Info);


	Control_LQR_U_Calculate(&Left_Control_Info);
  Control_LQR_U_Calculate(&Right_Control_Info);
	if(remote_ctrl.rc.s[0]!= 1){
	 Target_Angle -= remote_ctrl.rc.ch[0]*0.0002f;
}

	Left_Turn_T = f_PID_Calculate(&PID_Left_Turn_Velocity,f_PID_Calculate(&PID_Left_Turn_Angle,Target_Angle,INS_Info.yaw_tolangle),INS_Info.yaw_gyro);
  Right_Turn_T= f_PID_Calculate(&PID_Right_Turn_Velocity,f_PID_Calculate(&PID_Right_Turn_Angle,Target_Angle,INS_Info.yaw_tolangle),INS_Info.yaw_gyro);
	Left_Control_Info.Moment.T  =   Left_Control_Info.Moment.Stand_T  -Left_Turn_T;   // + Left_Control_Info.Moment.Turn_T);
  Right_Control_Info.Moment.T =  -Right_Control_Info.Moment.Stand_T -Right_Turn_T ;//+ Right_Control_Info.Moment.Turn_T;

	VAL_LIMIT(Right_Control_Info.Moment.T,-4.f,4.f);
	VAL_LIMIT(Left_Control_Info.Moment.T,-4.f,4.f);
		//if(remote_ctrl.rc.ch[0]!=0){
	Leg_Coordinate_Tp = f_PID_Calculate(&PID_Leg_Coordinate,0,Left_Control_Info.Measure.Theta-Right_Control_Info.Measure.Theta);
		//}else Leg_Coordinate_Tp = 0;
	Right_Control_Info.Moment.Tp =  Right_Control_Info.Moment.Tp ; //-Leg_Coordinate_Tp;
	Left_Control_Info.Moment.Tp  =  -Left_Control_Info.Moment.Tp ; //-Leg_Coordinate_Tp;
 F_Roll = f_PID_Calculate(&PID_Roll,0,INS_Info.rol_angle);
  VMC_F_Tp_To_Joint_Calculate(&Left_Control_Info,&Left_VMC_Info);
 VMC_F_Tp_To_Joint_Calculate1(&Right_Control_Info,&Right_VMC_Info);
 //  printf(L0,"%f,%f",Target_Chassis_Position,(Left_Control_Info.Measure.Chassis_Position+Right_Control_Info.Measure.Chassis_Position)/2);
   //printf(L0,"%f",ACC);
    osDelayUntil(&systick,2);
  }
}
  /* USER CODE END Control_Task */
static void VMC_Calculate(VMC_Info_Typedef *VMC_Info){
  VMC_Info->X_B = VMC_Info->L1 * arm_cos_f32(*VMC_Info->Phi1);
  VMC_Info->Y_B = VMC_Info->L1 * arm_sin_f32(*VMC_Info->Phi1);
	VMC_Info->X_D = VMC_Info->L5 + VMC_Info->L4 * arm_cos_f32(*VMC_Info->Phi4);
  VMC_Info->Y_D = VMC_Info->L4 * arm_sin_f32(*VMC_Info->Phi4);
	float X_D_Difference_X_B =  VMC_Info->X_D -  VMC_Info->X_B;
	float Y_D_Difference_Y_B =  VMC_Info->Y_D -  VMC_Info->Y_B;
  float LBD_2 =   X_D_Difference_X_B*X_D_Difference_X_B +Y_D_Difference_Y_B*Y_D_Difference_Y_B;
	float A0 =  2 * VMC_Info->L2  * X_D_Difference_X_B;
  float B0 =  2 * VMC_Info->L2  * Y_D_Difference_Y_B;
	float sqrt_out;
	arm_sqrt_f32(A0*A0 + B0*B0 - LBD_2*LBD_2,&sqrt_out);
  VMC_Info->Phi2 =2 * atan2f(B0+sqrt_out , A0 + LBD_2);
  VMC_Info->X_C  = VMC_Info->X_B + VMC_Info->L2 * arm_cos_f32(VMC_Info->Phi2);
  VMC_Info->Y_C  = VMC_Info->Y_B + VMC_Info->L2 * arm_sin_f32(VMC_Info->Phi2);
  VMC_Info->Phi3 = atan2f( VMC_Info->Y_C- VMC_Info->Y_D,VMC_Info->X_C- VMC_Info->X_D);
	arm_sqrt_f32(powf(VMC_Info->X_C - VMC_Info->L5/2,2) + VMC_Info->Y_C*VMC_Info->Y_C,&VMC_Info->L0);
	VMC_Info->Phi0 = atan2f(VMC_Info->Y_C,VMC_Info->X_C - (VMC_Info->L5/2));
	VMC_Info->Phi0_dot  = (VMC_Info->Phi0 - VMC_Info->Last_Phi0)/0.002f;
	VMC_Info->Last_Phi0 = VMC_Info->Phi0;
	//if(VMC_Info->VMC_USAGE == 1)  VMC_Info->Phi0 = 3.141593f -  VMC_Info->Phi0;
}
static void VMC_Calculate1(VMC_Info_Typedef *VMC_Info){
  VMC_Info->X_B = VMC_Info->L1 * arm_cos_f32(*VMC_Info->Phi1);
  VMC_Info->Y_B = VMC_Info->L1 * arm_sin_f32(*VMC_Info->Phi1);
	VMC_Info->X_D = VMC_Info->L5 + VMC_Info->L4 * arm_cos_f32(*VMC_Info->Phi4);
  VMC_Info->Y_D = VMC_Info->L4 * arm_sin_f32(*VMC_Info->Phi4);
	float X_D_Difference_X_B =  VMC_Info->X_D -  VMC_Info->X_B;
	float Y_D_Difference_Y_B =  VMC_Info->Y_D -  VMC_Info->Y_B;
  float LBD_2 =   X_D_Difference_X_B*X_D_Difference_X_B +Y_D_Difference_Y_B*Y_D_Difference_Y_B;
	float A0 =  2 * VMC_Info->L2  * X_D_Difference_X_B;
  float B0 =  2 * VMC_Info->L2  * Y_D_Difference_Y_B;
	float sqrt_out;
	arm_sqrt_f32(A0*A0 + B0*B0 - LBD_2*LBD_2,&sqrt_out);
  VMC_Info->Phi2 =2 * atan2f(B0+sqrt_out , A0 + LBD_2);
  VMC_Info->X_C  = VMC_Info->X_B + VMC_Info->L2 * arm_cos_f32(VMC_Info->Phi2);
  VMC_Info->Y_C  = VMC_Info->Y_B + VMC_Info->L2 * arm_sin_f32(VMC_Info->Phi2);
  VMC_Info->Phi3 = atan2f( VMC_Info->Y_C- VMC_Info->Y_D,VMC_Info->X_C- VMC_Info->X_D);
	arm_sqrt_f32(powf(VMC_Info->X_C - VMC_Info->L5/2,2) + VMC_Info->Y_C*VMC_Info->Y_C,&VMC_Info->L0);
// 	VMC_Info->L0_dot = (VMC_Info->L0-VMC_Info->Last_L0)/0.002f;
//	VMC_Info->Last_L0 =  VMC_Info->L0;
	VMC_Info->Phi0 = atan2f(VMC_Info->Y_C,VMC_Info->X_C - (VMC_Info->L5/2));
	VMC_Info->Phi0_dot  = (VMC_Info->Phi0 - VMC_Info->Last_Phi0)/0.002f;
	VMC_Info->Last_Phi0 = VMC_Info->Phi0;
	//if(VMC_Info->VMC_USAGE == 1)  VMC_Info->Phi0 = 3.141593f -  VMC_Info->Phi0;
}
static void Control_LQR_K_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info){
   float L0;//0.13;
	 L0 =VMC_Info->L0;
	Control_Info->LQR_K[0][0]=  a11[1]*powf(L0,3)   + a11[2]*powf(L0,2)    +a11[3]*L0    +a11[4];      
  Control_Info->LQR_K[0][1]=  a12[1]*powf(L0,3)   + a12[2]*powf(L0,2)    +a12[3]*L0    +a12[4];
  Control_Info->LQR_K[0][2]=  a13[1]*powf(L0,3)   + a13[2]*powf(L0,2)    +a13[3]*L0    +a13[4];
  Control_Info->LQR_K[0][3]=  a14[1]*powf(L0,3)   + a14[2]*powf(L0,2)    +a14[3]*L0    +a14[4];
  Control_Info->LQR_K[0][4]=  a15[1]*powf(L0,3)   + a15[2]*powf(L0,2)    +a15[3]*L0    +a15[4];
  Control_Info->LQR_K[0][5]=  a16[1]*powf(L0,3)   + a16[2]*powf(L0,2)    +a16[3]*L0    +a16[4];

	Control_Info->LQR_K[1][0]=   a21[1]*powf(L0,3)   + a21[2]*powf(L0,2)    +a21[3]*L0    +a21[4];     
  Control_Info->LQR_K[1][1]=   a22[1]*powf(L0,3)   + a22[2]*powf(L0,2)    +a22[3]*L0    +a22[4];
  Control_Info->LQR_K[1][2]=   a23[1]*powf(L0,3)   + a23[2]*powf(L0,2)    +a23[3]*L0    +a23[4];
  Control_Info->LQR_K[1][3]=   a24[1]*powf(L0,3)   + a24[2]*powf(L0,2)    +a24[3]*L0    +a24[4];
  Control_Info->LQR_K[1][4]=   a25[1]*powf(L0,3)   + a25[2]*powf(L0,2)    +a25[3]*L0    +a25[4];
  Control_Info->LQR_K[1][5]=   a26[1]*powf(L0,3)   + a26[2]*powf(L0,2)    +a26[3]*L0    +a26[4];
}

static void Control_Measure_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info){
//  if(Control_Info->mode == CHASSIS_WEAK)	{
   //VMC_Info->Phi0 = 1.5707965f;
//  }
	  Control_Info->Measure.Phi = INS_Info.angle[2];
    Control_Info->Measure.Phi_dot  = INS_Info.gyro[2]; //(Control_Info->Measure.Phi- Control_Info->Measure.Last_Phi)/0.002f; 
    Control_Info->Measure.Last_Phi =   Control_Info->Measure.Phi ;
	  
	  if(Control_Info->Control_USAGE == Right_Control){
    
		 Control_Info->Measure.Theta = ( VMC_Info->Phi0 - 1.5707965f) -Control_Info->Measure.Phi;
		}else{
		  Control_Info->Measure.Theta = ( 1.5707965f - VMC_Info->Phi0 )- Control_Info->Measure.Phi;
		  
	 }

    Control_Info->Measure.Theta_dot = (Control_Info->Measure.Theta - Control_Info->Measure.Last_Theta)/0.002f; //+ INS_Info.gyro[2];
	  Control_Info->Measure.Last_Theta = Control_Info->Measure.Theta;
	  if(Control_Info->mode == CHASSIS_BALANCE){ 
   	 
	  Control_Info->Measure.Chassis_Velocity = Measure_Chassis_Velocity;//;-(Control_Info->Measure.Phi_dot+VMC_Info->Phi0_dot)*0.08f;
		
			Control_Info->Measure.Chassis_Position += (Control_Info->Measure.Chassis_Velocity*0.002f);
	  
	 	}else{ 
	  Control_Info->Measure.Chassis_Velocity = 0;
		Control_Info->Measure.Chassis_Position = 0; 
	
  }
}
static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info){

	Control_Info->LQR_X[0] =( -0.04f - Control_Info->Measure.Theta);
	Control_Info->LQR_X[1] = (0 - Control_Info->Measure.Theta_dot);
	Control_Info->LQR_X[2] = (Target_Chassis_Position  - Control_Info->Measure.Chassis_Position) ;//
  Control_Info->LQR_X[3] = (Target_Chassis_Velocity  - Control_Info->Measure.Chassis_Velocity);
	Control_Info->LQR_X[4] = (0.04f - Control_Info->Measure.Phi);
  Control_Info->LQR_X[5] = (0 - Control_Info->Measure.Phi_dot);


}
static void Control_LQR_U_Calculate(Control_Info_Typedef *Control_Info){
 Control_Info->LQR_Output[0][0] = Control_Info->LQR_X[0]*Control_Info->LQR_K[0][0];
 Control_Info->LQR_Output[0][1] = Control_Info->LQR_X[1]*Control_Info->LQR_K[0][1];
 Control_Info->LQR_Output[0][2] = Control_Info->LQR_X[2]*Control_Info->LQR_K[0][2];
 Control_Info->LQR_Output[0][3] = Control_Info->LQR_X[3]*Control_Info->LQR_K[0][3];
 Control_Info->LQR_Output[0][4] = Control_Info->LQR_X[4]*Control_Info->LQR_K[0][4];
 Control_Info->LQR_Output[0][5] = Control_Info->LQR_X[5]*Control_Info->LQR_K[0][5];
// VAL_LIMIT( Control_Info->LQR_Output[0][0],-1.f,1.f);
// VAL_LIMIT( Control_Info->LQR_Output[0][1],-1.f,1.f);
// VAL_LIMIT( Control_Info->LQR_Output[0][2],-1.f,1.f);
// VAL_LIMIT( Control_Info->LQR_Output[0][3],-1.f,1.f);
// VAL_LIMIT( Control_Info->LQR_Output[0][4],-1.f,1.f);
// VAL_LIMIT( Control_Info->LQR_Output[0][5],-1.f,1.f);
Control_Info->Moment.Stand_T = (Control_Info->LQR_Output[0][0]+Control_Info->LQR_Output[0][1]+Control_Info->LQR_Output[0][2]+
	                         Control_Info->LQR_Output[0][3]+Control_Info->LQR_Output[0][4]+Control_Info->LQR_Output[0][5]);

VAL_LIMIT(Control_Info->Moment.Stand_T,-3.f,3.f);

 Control_Info->LQR_Output[1][0] = Control_Info->LQR_X[0]*Control_Info->LQR_K[1][0];
 Control_Info->LQR_Output[1][1] = Control_Info->LQR_X[1]*Control_Info->LQR_K[1][1];
 Control_Info->LQR_Output[1][2] = Control_Info->LQR_X[2]*Control_Info->LQR_K[1][2];
 Control_Info->LQR_Output[1][3] = Control_Info->LQR_X[3]*Control_Info->LQR_K[1][3];
 Control_Info->LQR_Output[1][4] = Control_Info->LQR_X[4]*Control_Info->LQR_K[1][4];
 Control_Info->LQR_Output[1][5] = Control_Info->LQR_X[5]*Control_Info->LQR_K[1][5];
// VAL_LIMIT( Control_Info->LQR_Output[1][0],-1.f,1.f);
// VAL_LIMIT( Control_Info->LQR_Output[1][1],-1.f,1.f);
// VAL_LIMIT( Control_Info->LQR_Output[1][2],-5.f,5.f);
// VAL_LIMIT( Control_Info->LQR_Output[1][3],-5.f,5.f);
// VAL_LIMIT( Control_Info->LQR_Output[1][4],-1.f,1.f);
// VAL_LIMIT( Control_Info->LQR_Output[1][5],-1.f,1.f);
	Control_Info->Moment.Tp = (Control_Info->LQR_Output[1][0]+Control_Info->LQR_Output[1][1]+Control_Info->LQR_Output[1][2]+
	                           Control_Info->LQR_Output[1][3]+Control_Info->LQR_Output[1][4]+Control_Info->LQR_Output[1][5]);
 
 VAL_LIMIT(Control_Info->Moment.Tp,-30.f,30.f);
}
static void VMC_F_Tp_To_Joint_Calculate(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info){

  if(Control_Info->mode == CHASSIS_WEAK){
	  init0 = 0;
	  qiankui0=0;
		VMC_Info->Target_L0 =0;
	}
	if(Control_Info->mode == CHASSIS_BALANCE&&init0==0){
	  VMC_Info->Target_L0 = f_Ramp_Calc(VMC_Info->Target_L0,0.185f,0.1f);
	  if( VMC_Info->Target_L0 ==0.185f ){
		 qiankui0 = f_Ramp_Calc(qiankui0,37,0.1f);
		 if(qiankui0 ==37) init0 =1;
		}
	} 
	if(remote_ctrl.rc.s[0]== 1){
	 VMC_Info->Target_L0 +=remote_ctrl.rc.ch[1]*0.000002f;
	 VAL_LIMIT( VMC_Info->Target_L0,0.185f,0.40f);
	}
 
	

  
	VMC_Info->F =  qiankui0+f_PID_Calculate(&PID_Leg_length_thrust[VMC_Info->VMC_USAGE],VMC_Info->Target_L0,VMC_Info->L0)
                        -F_Roll;
	VAL_LIMIT(	VMC_Info->F,-40.f,200.f);
  float Phi2_3 =  (  VMC_Info->Phi2 -VMC_Info->Phi3);
  float Phi0_3 =  ( VMC_Info->Phi0 -  VMC_Info->Phi3);
  float Phi1_2 =  ((*VMC_Info->Phi1) -  VMC_Info->Phi2);
   VMC_Info->T1 = -(((VMC_Info->L1 * arm_sin_f32(Phi1_2))*(VMC_Info->F * VMC_Info->L0 * arm_sin_f32(Phi0_3)+ 
	                 Control_Info->Moment.Tp * arm_cos_f32(Phi0_3)))/(VMC_Info->L0*arm_sin_f32(Phi2_3)));
  float Phi0_2 =  ( VMC_Info->Phi0 -  VMC_Info->Phi2);
  float Phi3_4 =  ( VMC_Info->Phi3 - (*VMC_Info->Phi4));
  VMC_Info->T2=  -(((VMC_Info->L4 * arm_sin_f32(Phi3_4))*(VMC_Info->F * VMC_Info->L0 * arm_sin_f32(Phi0_2)+ 
	                 Control_Info->Moment.Tp  * arm_cos_f32(Phi0_2)))/(VMC_Info->L0*arm_sin_f32(Phi2_3)));
	if( Control_Info->mode == CHASSIS_WEAK){
	 VMC_Info->T1 = 0;
	 VMC_Info->T2 = 0;
	}
 }
static void VMC_F_Tp_To_Joint_Calculate1(Control_Info_Typedef *Control_Info,VMC_Info_Typedef *VMC_Info){

  if(Control_Info->mode == CHASSIS_WEAK){
	  init1 = 0;
	  qiankui1=0;
		VMC_Info->Target_L0 =0;
	}
	if(Control_Info->mode == CHASSIS_BALANCE&&init1==0){
	  VMC_Info->Target_L0 = f_Ramp_Calc(VMC_Info->Target_L0,0.18f,0.1f);
	  if( VMC_Info->Target_L0 ==0.18f ){
		 qiankui1 = f_Ramp_Calc(qiankui1,37,0.1f);
		 if(qiankui1 ==37) init1 =1;
		}
	} 
	if(remote_ctrl.rc.s[0]== 1){
	 VMC_Info->Target_L0 +=remote_ctrl.rc.ch[1]*0.000002f;
	 VAL_LIMIT( VMC_Info->Target_L0,0.18f,0.39f);
	}
 
	

  
	VMC_Info->F = qiankui1+ f_PID_Calculate(&PID_Leg_length_thrust[VMC_Info->VMC_USAGE],VMC_Info->Target_L0,VMC_Info->L0)
                          +F_Roll; 
  
  float Phi2_3 =  (  VMC_Info->Phi2 -VMC_Info->Phi3);
  float Phi0_3 =  ( VMC_Info->Phi0 -  VMC_Info->Phi3);
  float Phi1_2 =  ((*VMC_Info->Phi1) -  VMC_Info->Phi2);
   VMC_Info->T1 = -(((VMC_Info->L1 * arm_sin_f32(Phi1_2))*(VMC_Info->F * VMC_Info->L0 * arm_sin_f32(Phi0_3)+ 
	                 Control_Info->Moment.Tp * arm_cos_f32(Phi0_3)))/(VMC_Info->L0*arm_sin_f32(Phi2_3)));
  float Phi0_2 =  ( VMC_Info->Phi0 -  VMC_Info->Phi2);
  float Phi3_4 =  ( VMC_Info->Phi3 - (*VMC_Info->Phi4));
  VMC_Info->T2=  -(((VMC_Info->L4 * arm_sin_f32(Phi3_4))*(VMC_Info->F * VMC_Info->L0 * arm_sin_f32(Phi0_2)+ 
	                 Control_Info->Moment.Tp  * arm_cos_f32(Phi0_2)))/(VMC_Info->L0*arm_sin_f32(Phi2_3)));
	if( Control_Info->mode == CHASSIS_WEAK){
	 VMC_Info->T1 = 0;
	 VMC_Info->T2 = 0;
	}
 }