#ifndef _PROTRACT_H
#define _PROTRACT_H
#include <stm32h7xx_hal.h>
#include "motor.h"
#include "pid.h"
#include <dr16.h>

struct Protract_Motor_Info {
	float Target_Angle;
	float Can_Angle;
	float Last_Can_Angle;
	float Can_Angle_speed;
	float	angle;
	float	angle_last;
	int16_t Can_Speed_Feedback;
	float Sensitivity;
	uint16_t ResetCount;
};

typedef struct {
	struct Protract_Motor_Info Protract_left;
	struct Protract_Motor_Info Protract_right;
	struct {
		Motor motor[2];						//< 姿态电机结构体
		BasePID_Object turnPID[6];		//< 转向角度控制结构体
		float FeedbackAngle[2];
		float HolderCanAngle[2];
	} Motors3508;

} Protract_t;

void ProtractInit(Protract_t *protract, BasePID_Object pid_left_angle, BasePID_Object pid_left_speed,
                  BasePID_Object pid_right_angle, BasePID_Object pid_right_speed, BasePID_Object pid_leftreset,
                  BasePID_Object pid_rightreset, CanNumber canx);
void ProtractReset(Protract_t *protract);
void ProtractGetRemoteData(Protract_t *protract, RC_Ctrl *rc_ctrl);
void Protract_Ore(Protract_t *protract, RC_Ctrl *rc_ctrl,float target);
void air_pressure_switch(RC_Ctrl *rc_ctrl);
extern uint16_t resetcount1;
extern uint8_t protract_reset_flag;

extern  int direction_flag;
extern 	int target;


#endif



