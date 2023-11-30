#ifndef VMC_Task_H
#define VMC_Task_H

#include "stdint.h"










typedef struct 
{
    float L1,L2,L3,L4,L5,LBD,L0,Target_L0,Last_Target_L0;
	float *Phi1,*Phi4;
	float  Phi2,Phi3,Phi0;
	float  *Predict_Phi1,*Predict_Phi4,Predict_Phi2,Predict_Phi0;
	float *Phi1_dot,*Phi4_dot,Phi0_dot,Last_Phi0;
	float X_B,Y_B,X_D,Y_D,X_C,Y_C,Y_C_dot,X_C_dot;
	float F,T1,T2,Tp;
	float Theta;
	float Predict_Theta;
	uint8_t Leg_length_Change;
}VMC_Info_Typedef;
typedef struct 
{
 float Phi1,Phi2,Phi3,Phi4,Phi0;  

}VMC_Rad_to_Angle_Info_Typedef;

extern VMC_Info_Typedef  Left_VMC_Info;
extern VMC_Info_Typedef  Right_VMC_Info;

extern  VMC_Rad_to_Angle_Info_Typedef VMC_Rad_to_Angle_Info;





#endif //CONTROLLER_PID_H
