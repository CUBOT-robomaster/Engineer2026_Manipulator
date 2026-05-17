#ifndef _SIDEWAY_H
#define _SIDEWAY_H
#include <stm32h7xx_hal.h>
#include "motor.h"
#include "pid.h"
#include <dr16.h>

struct Sideway_Motor_Info {
	float Target_Angle;
	float Can_Angle;
	float Last_Can_Angle;
	float Can_Angle_speed;
	float	angle;
	float	angle_last;
	int16_t Can_Speed_Feedback;
	float Sensitivity;
};

typedef struct {
	struct Sideway_Motor_Info Sideway;

	struct {
		Motor motor[2];						//< 姿态电机结构体
		BasePID_Object turnPID[6];		//< 转向角度控制结构体
		float FeedbackAngle[2];
		float HolderCanAngle[2];
	} Motors3508;

} Sideway_t;

void SidewayInit(Sideway_t *sideway, BasePID_Object pid_left_angle, BasePID_Object pid_left_speed,  BasePID_Object pid_leftreset, CanNumber canx); 
void SidewayReset(Sideway_t *sideway);
void SidewayGetRemoteData(Sideway_t *sideway, RC_Ctrl *rc_ctrl);
#endif



