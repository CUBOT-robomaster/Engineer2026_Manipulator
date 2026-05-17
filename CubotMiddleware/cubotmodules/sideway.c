#include "sideway.h"
#include "hardware_config.h"
#include <dr16.h>
float left_sideway_limit = -3000;
float right_sideway_limt = 1000;

float sideway_vel_param = 1.0;

void SidewayInit(Sideway_t *sideway, BasePID_Object pid_left_angle, BasePID_Object pid_left_speed,  BasePID_Object pid_leftreset, CanNumber canx) 
{
	MotorInit(&sideway->Motors3508.motor[0],  7702, Motor3508, canx, 0x203);

	BasePID_Init(&sideway->Motors3508.turnPID[0], pid_left_angle.Kp, pid_left_angle.Ki, pid_left_angle.Kd, pid_left_angle.KiPartDetachment);
	BasePID_Init(&sideway->Motors3508.turnPID[1], pid_left_speed.Kp, pid_left_speed.Ki, pid_left_speed.Kd, pid_left_speed.KiPartDetachment);
	BasePID_Init(&sideway->Motors3508.turnPID[2], pid_leftreset.Kp, pid_leftreset.Ki, pid_leftreset.Kd, pid_leftreset.KiPartDetachment);
}

void SidewayReset(Sideway_t *sideway) 
{
	sideway->Motors3508.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object *)(sideway->Motors3508.turnPID + 2), 0, sideway->Motors3508.motor[0].Data.Angle, sideway->Motors3508.motor[0].Data.SpeedRPM);

	for (int i = 0 ; i < 1 ; i ++) 
	{
		if (sideway->Motors3508.motor[i].Data.Output > 30000)
				sideway->Motors3508.motor[i].Data.Output = 30000;
		if (sideway->Motors3508.motor[i].Data.Output < -30000)
				sideway->Motors3508.motor[i].Data.Output = -30000;
	}
	MotorFillData(&sideway->Motors3508.motor[0], sideway->Motors3508.motor[0].Data.Output);
}

//说明：从3508的减速箱方向看，参数符号为负时顺时针旋转，反之可推
//从车的后侧看，横移机构顺时针旋转的时候左移，反之可推
void SidewayGetRemoteData(Sideway_t *sideway, RC_Ctrl *rc_ctrl) {
//     attitude->Yaw.Target_Angle += (rc_ctrl->rc.ch2 -1024)* attitude->Yaw.Sensitivity;
//    attitude->Pitch.Target_Angle += (rc_ctrl->rc.ch3 -1024)* attitude->Pitch.Sensitivity;
	if ((rc_ctrl->key_E_flag==1) && (rc_ctrl->isOnline == 1)) 
	{
		sideway->Sideway.Target_Angle += sideway_vel_param;
	} 
	else if ((rc_ctrl->key_E_flag==0&&rc_ctrl->key_F_flag==0) && (rc_ctrl->isOnline == 1)) 
	{
		sideway->Sideway.Target_Angle = sideway->Sideway.Target_Angle;
	} 
	else if ((rc_ctrl->key_F_flag==1) && (rc_ctrl->isOnline == 1)) 
	{
		sideway->Sideway.Target_Angle -= sideway_vel_param;
	}
	
	//运动范围限幅
	if (sideway->Sideway.Target_Angle <= left_sideway_limit)
		sideway->Sideway.Target_Angle = left_sideway_limit;
	else if (sideway->Sideway.Target_Angle >= right_sideway_limt)
		sideway->Sideway.Target_Angle = right_sideway_limt;

	if ((sideway->Motors3508.motor[0].Data.Angle < -140) && (sideway->Sideway.angle_last > 140)) {
		sideway->Sideway.angle += 360 + sideway->Motors3508.motor[0].Data.Angle - sideway->Sideway.angle_last;
	} else if (( sideway->Motors3508.motor[0].Data.Angle > 140) && (sideway->Sideway.angle_last < -140)) {
		sideway->Sideway.angle += -360 + sideway->Motors3508.motor[0].Data.Angle - sideway->Sideway.angle_last;
	} else {
		sideway->Sideway.angle += sideway->Motors3508.motor[0].Data.Angle - sideway->Sideway.angle_last;
	}
	sideway->Sideway.angle_last = sideway->Motors3508.motor[0].Data.Angle;


//	if ((sideway->Motors3508.motor[1].Data.Angle < -140) && (sideway->Sideway.angle_last > 140)) {
//		sideway->Sideway.angle += 360 + sideway->Motors3508.motor[1].Data.Angle - sideway->Sideway.angle_last;
//	} else if (( sideway->Motors3508.motor[1].Data.Angle > 140) && (sideway->Sideway.angle_last < -140)) {
//		sideway->Sideway.angle += -360 + sideway->Motors3508.motor[1].Data.Angle - sideway->Sideway.angle_last;
//	} else {
//		sideway->Sideway.angle += sideway->Motors3508.motor[1].Data.Angle - sideway->Sideway.angle_last;
//	}
//	sideway->Sideway.angle_last = sideway->Motors3508.motor[1].Data.Angle;

//	sideway->Motors3508.motor[0].Data.Output = BasePID_PitchSpeedControl((BasePID_Object *)(
//	            sideway->Motors3508.turnPID + 1), BasePID_PitchAngleControl((BasePID_Object *)(sideway->Motors3508.turnPID + 0),
//	                    sideway->Sideway.Target_Angle, sideway->Sideway.angle), sideway->Motors3508.motor[0].Data.SpeedRPM);
//	sideway->Motors3508.motor[1].Data.Output = BasePID_YawSpeedControl((BasePID_Object *)(
//	            sideway->Motors3508.turnPID + 3), BasePID_YawAngleControl((BasePID_Object *)(sideway->Motors3508.turnPID + 2),
//	                    sideway->Protract_right.Target_Angle, sideway->Protract_right.angle), sideway->Motors3508.motor[1].Data.SpeedRPM);
      sideway->Motors3508.motor[0].Data.Output  =  BasePID_SidewaySpeedControl((BasePID_Object *)(sideway->Motors3508.turnPID + 1), BasePID_SidewayAngleControl((BasePID_Object *)(sideway->Motors3508.turnPID + 0),sideway->Sideway.Target_Angle, sideway->Sideway.angle), sideway->Motors3508.motor[0].Data.SpeedRPM);
	if (sideway->Motors3508.motor[0].Data.Output >= 29000)
		  sideway->Motors3508.motor[0].Data.Output = 29000;
	if (sideway->Motors3508.motor[0].Data.Output <= -29000)
	  	sideway->Motors3508.motor[0].Data.Output = -29000;

	MotorFillData(&sideway->Motors3508.motor[0], sideway->Motors3508.motor[0].Data.Output);
}