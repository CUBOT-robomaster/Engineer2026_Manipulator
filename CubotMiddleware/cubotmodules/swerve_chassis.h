#ifndef SWERVECHASSIS_H
#define SWERVECHASSIS_H

#include "stm32h7xx.h"
#include "pid.h"
#include "motor.h"
#include <dr16.h>
typedef struct 
{
	struct
	{
		uint8_t Enable;    						//< 使能状态			
		uint8_t isRefereeUpdating; 		//< 裁判系统是否在更新数据
		uint8_t DriveMode;						//< 操作模式
		uint8_t ChassisState;					//< 底盘状态
	}Flag;
	
	struct 
	{
		float Target_Power_Sum[2];		
		BasePID_Object PowerPID[2];
		float Max_Power;		
    float speedfinalout[4];
    float anglefinalout[4];	
		float bili;
		float bilibili;
	}Power;
	
	struct 
	{
		Motor   motor[4];						//< 底盘电机结构体
		BasePID_Object RunPID[4];			//< 速度控制PID参数
		BasePID_Object FollowPID[1];			//< 底盘跟随PID参数
	}Motors3508;
	
	struct 
	{
		Motor motor[4];						//< 底盘电机结构体
		BasePID_Object TurnPID[4];			//< 转向角度控制结构体
	}Motors6020;
	
	//< 控制底盘运动所需要的所有数据
	struct 
	{
		int16_t Vx;			//< 前后运动的速度
		int16_t Vy;		  	//< 左右运动的速度
		int16_t Omega;		//< 旋转的角速度
	  int16_t		DeltaVx;
	  int16_t		DeltaVy;
	  int16_t		DeltaOmega;
		struct 
		{
			float		Vx;
			float		Vy;
			float		Omega;
		}Sensitivity;
		
		int16_t ModuleOfSpeed;		//< 速度向量的模值
		float  AngleOfSpeed;		//< 速度向量的角度
	}Movement;  
	
	struct 
	{
		int16_t Vx[4];
		int16_t Vy[4];
		float Angle[4];	
		float BestAngle[4];
		int16_t TargetEcd[4];
		float FeedbackAngle[4];
		int16_t SpeedNo[4];
		uint8_t SpeedChangeFlag[4];
	}Vectors;
	uint8_t SuperPowerMode;
}SwerveChassis;


void SwerveChassisMotionControl(SwerveChassis* chassis);
void VectorSolve(int16_t vx, int16_t vy, float* vectorAngle, int16_t* vectorModule, uint8_t id);
float EcdtoAngle(int16_t offset, int16_t ecd);
void SwerveChassisInit(SwerveChassis* chassis, BasePID_Object run_pid, BasePID_Object turn_pid, CanNumber canx);
float FindBestTargetAngle(float targetAngle, float feedbackAngle, uint8_t* flag);
void SwerveChassisGetRemoteData(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,float canAngle);
void SwerveChassisMotionControl(SwerveChassis* chassis);
void SwerveChassisOutputControl(SwerveChassis* chassis, RC_Ctrl rcCtrl);

#endif









