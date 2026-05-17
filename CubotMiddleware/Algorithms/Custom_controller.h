#ifndef _CUSTOM_CONTROLLER_H
#define _CUSTOM_CONTROLLER_H
#include <stm32h7xx_hal.h>
#include "motor.h"
#include "pid.h"

#define pi 3.141592653589
typedef struct
{
	struct 
	{
		Motor motor[3];	
		BasePID_Object turn_pid[6];
		float angle[3];
		float deg_last[3];
		float angle_last[3];
		float target_angle[3];
	}Data_3508; //3508电机的数据结构体
	struct 
	{
		Motor motor[3];	
		BasePID_Object turn_pid[6];
		float angle[3];
		float deg_last[3];
		float angle_last[3];
		float target_angle[3];
	}Data_6020; //3508电机的数据结构体
	struct 
	{
		float pos_x;
		float pos_y;
		float pos_z;
		float pos_x_speed;
		float pos_y_speed;
		float pos_z_speed;
		float yaw;
		float pitch1;
		float pitch2;
		float roll1;
		float pitch3;
		float roll2;
	}DP;  //
}Controller_s;


void Controller_Init(Controller_s *Controller,BasePID_Object damp_pid);
void Controller_Dataprocs(Controller_s *Controller);
void Damp_test(Controller_s *Controller);
void speed_caculator(Controller_s *Controller);
void Controller_6020_Init_v1(Controller_s *Controller, BasePID_Object yaw_angle_pid ,BasePID_Object yaw_speed_pid, BasePID_Object yaw2_angle_pid ,BasePID_Object yaw2_speed_pid,BasePID_Object yaw3_angle_pid ,BasePID_Object yaw3_speed_pid, CanNumber canx);
void Controller_3508_Init_v1(Controller_s *Controller, BasePID_Object yaw_angle_pid ,BasePID_Object yaw_speed_pid, BasePID_Object yaw2_angle_pid ,BasePID_Object yaw2_speed_pid,BasePID_Object yaw3_angle_pid ,BasePID_Object yaw3_speed_pid, CanNumber canx);
void Damp_test_v1(Controller_s *Controller);
float rad_deg(float rad);
float deg_rad(float deg);
void uncoil_coordinate(Controller_s *Controller);

#endif