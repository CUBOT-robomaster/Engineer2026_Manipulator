//#ifndef _LIFTING_H
//#define _LIFTING_H
//#include <stm32h7xx_hal.h>
//#include "motor.h"
//#include "pid.h"
//#include<dr16.h>

//extern float lifting_errortest;


//struct Lifting_Motor_Info
//{
//  float Target_Angle;
//	float Can_Angle;
//	float Last_Can_Angle;
//	float Can_Angle_speed;
//  float	angle;
//	float	angle_last;
//	int16_t Can_Speed_Feedback;
//	float Sensitivity;
//};
//typedef struct
//{
//	uint8_t Auto_State;
//	uint8_t Reset_OK_Flag;
//	uint8_t ESC_Reset_flag;
//	double lifting_target_1;
//	double lifting_target_2;

//	
//  struct Lifting_Motor_Info Lifting_left_1;
//  struct Lifting_Motor_Info Lifting_right_1;
//	
//	struct Lifting_Motor_Info Lifting_left_2;
//  struct Lifting_Motor_Info Lifting_right_2;
//	
//	struct 
//	{
//		Motor motor[4];						//< 姿态电机结构体
//		BasePID_Object turnPID[10];		//< 转向角度控制结构体
//		float FeedbackAngle[2];
//		float HolderCanAngle[2];
//	}Motors3508;
//	
//}Lifting_t;
////抬升机构初始化函数声明
//void Lifting_first_Init(Lifting_t *lifting,BasePID_Object pid_left_1_angle,BasePID_Object pid_left_1_speed,BasePID_Object pid_right_1_angle,BasePID_Object pid_right_1_speed,CanNumber canx);
//void Lifting_second_Init(Lifting_t *lifting,BasePID_Object pid_left_2_angle,BasePID_Object pid_left_2_speed,BasePID_Object pid_right_2_angle,BasePID_Object pid_right_2_speed,BasePID_Object reset_left,BasePID_Object reset_right,CanNumber canx);
//void Lifting_Reset(Lifting_t *lifting);
//void LiftingGetRemoteData(Lifting_t *lifting,RC_Ctrl *rc_ctrl);
//void Lifting_Ore_firstpart(Lifting_t *lifting,RC_Ctrl *rc_ctrl,float lifting_move_distance);
//void Lifting_Ore_secondpart(Lifting_t *lifting,RC_Ctrl *rc_ctrl,float lifting_move_distance);
//void Lifting_Ore_secondpart_reset(Lifting_t *lifting,RC_Ctrl *rc_ctrl,float lifting_move_distance);
//void plugging_reset(RC_Ctrl *rc_ctrl,Lifting_t *lifting);
//void plugging_reset1(RC_Ctrl *rc_ctrl,Lifting_t *lifting);

//extern float lifting_vel_param;   //6//5
//extern uint8_t plugging_reset_flag;
//extern uint8_t plugging_reset_flag1;

//#endif



