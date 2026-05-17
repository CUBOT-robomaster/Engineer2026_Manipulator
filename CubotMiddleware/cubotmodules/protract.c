#include "protract.h"
#include "hardware_config.h"
#include <dr16.h>
#include<driver_timer.h>
#include<math.h>
float left_forward_limit = 0;
float right_forward_limt = -20000;  //28000
float left_back_limit = 20000;
float right_back_limt = 0;
float protract_vel_param = 10;			  //遥控器控制-前伸速度参数//10
float protract_ore_vel_param = 18;	//25  //直接位置控制-前伸速度参数
float pro_leftsince=1;
float pro_rightsince=1;
float protract_param=11.21;
uint8_t protract_reset_flag=0;
int16_t error1;
int16_t error2;
uint8_t resetflag=0;
uint16_t resetlimit=1000;
uint16_t resetcount=0;
uint16_t resetcount1=0;
void ProtractInit(Protract_t *protract, BasePID_Object pid_left_angle,BasePID_Object pid_left_speed,BasePID_Object pid_right_angle,BasePID_Object pid_right_speed,BasePID_Object pid_leftreset,BasePID_Object pid_rightreset,CanNumber canx)
{
	MotorInit(&protract->Motors3508.motor[0],  2742, Motor3508, canx, 0x207);
	MotorInit(&protract->Motors3508.motor[1], 615, Motor3508, canx, 0x208);

	BasePID_Init(&protract->Motors3508.turnPID[0], pid_left_angle.Kp, pid_left_angle.Ki, pid_left_angle.Kd, pid_left_angle.KiPartDetachment);
	BasePID_Init(&protract->Motors3508.turnPID[1], pid_left_speed.Kp, pid_left_speed.Ki, pid_left_speed.Kd, pid_left_speed.KiPartDetachment);
	BasePID_Init(&protract->Motors3508.turnPID[2], pid_right_angle.Kp, pid_right_angle.Ki, pid_right_angle.Kd,pid_right_angle.KiPartDetachment);
	BasePID_Init(&protract->Motors3508.turnPID[3], pid_right_speed.Kp, pid_right_speed.Ki, pid_right_speed.Kd,pid_right_speed.KiPartDetachment);
	BasePID_Init(&protract->Motors3508.turnPID[4], pid_leftreset.Kp, pid_leftreset.Ki, pid_leftreset.Kd,pid_leftreset.KiPartDetachment);
	BasePID_Init(&protract->Motors3508.turnPID[5], pid_rightreset.Kp, pid_rightreset.Ki, pid_rightreset.Kd,pid_rightreset.KiPartDetachment);

}

void ProtractReset(Protract_t *protract) 
{
	  error1=protract->Motors3508.motor[1].Data.SpeedRPM-200;
	  error2=protract->Motors3508.motor[0].Data.SpeedRPM-200;
	if(error1>0)error1=error1;
	if(error1<0)error1=-error1;
	if(error2>0)error2=error2;
	if(error2<0)error2=-error2;
// if(((protract->Motors3508.motor[1].Data.SpeedRPM<200)&&(protract->Motors3508.motor[1].Data.SpeedRPM>-200))&&((protract->Motors3508.motor[0].Data.SpeedRPM<200)&&(protract->Motors3508.motor[0].Data.SpeedRPM>-200))&&(tim14.ClockTime>500))
if((error1<600)&&(error2<600)&&(tim14.ClockTime>100)&&(resetcount1>2000))	
	{
	   resetcount++;
		if(resetcount>200)
		{
		protract->Motors3508.motor[0].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(protract->Motors3508.turnPID + 4),0,protract->Motors3508.motor[0].Data.SpeedRPM);
		protract->Motors3508.motor[1].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(protract->Motors3508.turnPID + 5),0 ,protract->Motors3508.motor[1].Data.SpeedRPM);
		  MotorFillData(&protract->Motors3508.motor[1], protract->Motors3508.motor[0].Data.Output);
     	MotorFillData(&protract->Motors3508.motor[0], protract->Motors3508.motor[0].Data.Output);
	    MotorCanOutput(can1, 0x1ff);
		  MotorCanOutput(can1, 0x200);
			resetcount=200;
			protract_reset_flag=1;
		}
 }
   else 
	{
	  protract->Motors3508.motor[0].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(protract->Motors3508.turnPID + 4),-resetlimit,protract->Motors3508.motor[0].Data.SpeedRPM);
		protract->Motors3508.motor[1].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(protract->Motors3508.turnPID + 5),resetlimit,protract->Motors3508.motor[1].Data.SpeedRPM);
	if (protract->Motors3508.motor[0].Data.Output >= 4000)
		protract->Motors3508.motor[0].Data.Output = 4000;
	if (protract->Motors3508.motor[0].Data.Output <= -4000)
		protract->Motors3508.motor[0].Data.Output = -4000;
	if (protract->Motors3508.motor[1].Data.Output >= 4000)
		protract->Motors3508.motor[1].Data.Output = 4000;
	if (protract->Motors3508.motor[1].Data.Output <= -4000)
		protract->Motors3508.motor[1].Data.Output = -4000;
		MotorFillData(&protract->Motors3508.motor[1], protract->Motors3508.motor[1].Data.Output);
   	MotorFillData(&protract->Motors3508.motor[0], protract->Motors3508.motor[0].Data.Output);
	  MotorCanOutput(can1, 0x1ff);
		MotorCanOutput(can1, 0x200);
	  resetcount1++;
		tim14.ClockTime=101;
	}
}

//说明：从3508的减速箱方向看，参数符号为负时顺时针旋转，反之可推
//从车的后侧看，前伸机构左侧电机逆时针转，右侧电机顺时针转时前伸，反之可推
void ProtractGetRemoteData(Protract_t *protract, RC_Ctrl *rc_ctrl) {
//     attitude->Yaw.Target_Angle += (rc_ctrl->rc.ch2 -1024)* attitude->Yaw.Sensitivity;
//    attitude->Pitch.Target_Angle += (rc_ctrl->rc.ch3 -1024)* attitude->Pitch.Sensitivity;
//	if ((rc_ctrl->rc.s2 == 1) && (rc_ctrl->isOnline == 1)) 
		if(((rc_ctrl->rc.s2==1)||((rc_ctrl->key_shift_flag==1)&&(rc_ctrl->key_Z_flag==1)))&&(rc_ctrl->isOnline==1)&&(rc_ctrl->key_ctrl_flag==0))
		{
		protract->Protract_left.Target_Angle += protract_vel_param;
		protract->Protract_right.Target_Angle += -protract_vel_param;
		} 
//		else if ((rc_ctrl->rc.s2 == 3) && (rc_ctrl->isOnline == 1)) 
//		{
//		protract->Protract_left.Target_Angle = protract->Protract_left.Target_Angle;
//		protract->Protract_right.Target_Angle = protract->Protract_right.Target_Angle;
//		} 
		else if(((rc_ctrl->rc.s2==2)||((rc_ctrl->key_shift_flag==1)&&(rc_ctrl->key_X_flag==1)))&&(rc_ctrl->isOnline==1)&&(rc_ctrl->key_ctrl_flag==0)) 
		{
		protract->Protract_left.Target_Angle += -protract_vel_param;
		protract->Protract_right.Target_Angle += +protract_vel_param;
		}
		if(rc_ctrl->isOnline==0)
			{
			 protract->Protract_left.Target_Angle=protract->Protract_left.angle;
			 protract->Protract_right.Target_Angle=protract->Protract_right.angle;
			}
	
		
		//运动范围限幅
    if(protract->Protract_left.Target_Angle <= left_forward_limit)
    protract->Protract_left.Target_Angle = left_forward_limit;
    else if(protract->Protract_left.Target_Angle >= left_back_limit)
    protract->Protract_left.Target_Angle = left_back_limit;

		if(protract->Protract_right.Target_Angle <= right_forward_limt)
    protract->Protract_right.Target_Angle = right_forward_limt;
    else if(protract->Protract_right.Target_Angle >= right_back_limt)
    protract->Protract_right.Target_Angle = right_back_limt;

	if((protract->Motors3508.motor[0].Data.Angle < -100) && (protract->Protract_left.angle_last > 100)) 
	{
		protract->Protract_left.angle += 360 + protract->Motors3508.motor[0].Data.Angle - protract->Protract_left.angle_last;
			
	} 
	else if (( protract->Motors3508.motor[0].Data.Angle > 100) && (protract->Protract_left.angle_last < -100)) 
	{
		protract->Protract_left.angle += -360 + protract->Motors3508.motor[0].Data.Angle - protract->Protract_left.angle_last;
	}
	else 
	{
		protract->Protract_left.angle += protract->Motors3508.motor[0].Data.Angle - protract->Protract_left.angle_last;
	}
	  protract->Protract_left.angle_last = protract->Motors3508.motor[0].Data.Angle;


	if((protract->Motors3508.motor[1].Data.Angle < -100) && (protract->Protract_right.angle_last > 100)) 
	{
		protract->Protract_right.angle += 360 + protract->Motors3508.motor[1].Data.Angle - protract->Protract_right.angle_last;
	} 
	else if (( protract->Motors3508.motor[1].Data.Angle > 100) && (protract->Protract_right.angle_last < -100)) 
	{
		protract->Protract_right.angle += -360 + protract->Motors3508.motor[1].Data.Angle - protract->Protract_right.angle_last;
	} 
	else
	{
		protract->Protract_right.angle += protract->Motors3508.motor[1].Data.Angle - protract->Protract_right.angle_last;
	}
	  protract->Protract_right.angle_last = protract->Motors3508.motor[1].Data.Angle;

//	protract->Protract_left.angle+=protract->Motors3508.motor[0].Data.SpeedRPM*0.001*pro_leftsince;
//	protract->Protract_right.angle+=protract->Motors3508.motor[1].Data.SpeedRPM*0.001*pro_rightsince;
	
	protract->Motors3508.motor[0].Data.Output = BasePID_PitchSpeedControl((BasePID_Object *)(protract->Motors3508.turnPID + 1), BasePID_PitchAngleControl((BasePID_Object *)(protract->Motors3508.turnPID + 0),protract->Protract_left.Target_Angle, protract->Protract_left.angle), protract->Motors3508.motor[0].Data.SpeedRPM);
	protract->Motors3508.motor[1].Data.Output = BasePID_YawSpeedControl((BasePID_Object *)(protract->Motors3508.turnPID + 3), BasePID_YawAngleControl((BasePID_Object *)(protract->Motors3508.turnPID + 2),protract->Protract_right.Target_Angle, protract->Protract_right.angle), protract->Motors3508.motor[1].Data.SpeedRPM);

	if (protract->Motors3508.motor[0].Data.Output >= 29000)
		protract->Motors3508.motor[0].Data.Output = 29000;
	if (protract->Motors3508.motor[0].Data.Output <= -29000)
		protract->Motors3508.motor[0].Data.Output = -29000;
	if (protract->Motors3508.motor[1].Data.Output >= 30000)
		protract->Motors3508.motor[1].Data.Output = 30000;
	if (protract->Motors3508.motor[1].Data.Output <= -30000)
		protract->Motors3508.motor[1].Data.Output = -30000;

	MotorFillData(&protract->Motors3508.motor[1], protract->Motors3508.motor[1].Data.Output);
	MotorFillData(&protract->Motors3508.motor[0], protract->Motors3508.motor[0].Data.Output);
}


//前伸直接位置控制函数
//说明：protract_move_distance是机构实际运动的距离，单位为mm，target是换算之后发送的目标值
void Protract_Ore(Protract_t *protract, RC_Ctrl *rc_ctrl,float protract_move_distance) {

	int target;
	target=protract_move_distance*protract_param*10;			//换算公式
	int direction_flag=0;
 if(rc_ctrl->isOnline == 1)
  {
   if(protract->Protract_left.Target_Angle<=target)  //比较目标值和当前角度的大小，判断运动的方向
    direction_flag=0;
   else
    direction_flag=1;
   
   switch(direction_flag)
   {
    case 0:
     if(protract->Protract_left.Target_Angle<=target)
       {
      protract->Protract_left.Target_Angle += protract_ore_vel_param;
      protract->Protract_right.Target_Angle += -protract_ore_vel_param;
     }
     else
     {
      protract->Protract_left.Target_Angle = target;
      protract->Protract_right.Target_Angle = -target;
      }
     break;
    case 1:
     if(protract->Protract_left.Target_Angle>=target)
       {
      protract->Protract_left.Target_Angle +=-protract_ore_vel_param;
      protract->Protract_right.Target_Angle += protract_ore_vel_param;
     }
     else
     {
      protract->Protract_left.Target_Angle = target;
      protract->Protract_right.Target_Angle = -target;
      }     
   }
	}
		if(rc_ctrl->isOnline==0)
		{
		protract->Protract_left.Target_Angle =protract->Protract_left.angle;
		protract->Protract_right.Target_Angle =protract->Protract_right.angle;
		} 
		
		//运动范围限幅
    if(protract->Protract_left.Target_Angle <= left_forward_limit)
    protract->Protract_left.Target_Angle = left_forward_limit;
    else if(protract->Protract_left.Target_Angle >= left_back_limit)
    protract->Protract_left.Target_Angle = left_back_limit;

		if(protract->Protract_right.Target_Angle <= right_forward_limt)
    protract->Protract_right.Target_Angle = right_forward_limt;
    else if(protract->Protract_right.Target_Angle >= right_back_limt)
    protract->Protract_right.Target_Angle = right_back_limt;

	if ((protract->Motors3508.motor[0].Data.Angle < -100) && (protract->Protract_left.angle_last > 100)) {
		protract->Protract_left.angle += 360 + protract->Motors3508.motor[0].Data.Angle - protract->Protract_left.angle_last;
	} else if (( protract->Motors3508.motor[0].Data.Angle > 100) && (protract->Protract_left.angle_last < -100)) {
		protract->Protract_left.angle += -360 + protract->Motors3508.motor[0].Data.Angle - protract->Protract_left.angle_last;
	} else {
		protract->Protract_left.angle += protract->Motors3508.motor[0].Data.Angle - protract->Protract_left.angle_last;
	}
	protract->Protract_left.angle_last = protract->Motors3508.motor[0].Data.Angle;


	if ((protract->Motors3508.motor[1].Data.Angle < -100) && (protract->Protract_right.angle_last > 100)) {
		protract->Protract_right.angle += 360 + protract->Motors3508.motor[1].Data.Angle - protract->Protract_right.angle_last;
	} else if (( protract->Motors3508.motor[1].Data.Angle > 100) && (protract->Protract_right.angle_last < -100)) {
		protract->Protract_right.angle += -360 + protract->Motors3508.motor[1].Data.Angle - protract->Protract_right.angle_last;
	} else {
		protract->Protract_right.angle += protract->Motors3508.motor[1].Data.Angle - protract->Protract_right.angle_last;
	}
	protract->Protract_right.angle_last = protract->Motors3508.motor[1].Data.Angle;

	protract->Motors3508.motor[0].Data.Output = BasePID_PitchSpeedControl((BasePID_Object *)(protract->Motors3508.turnPID + 1), BasePID_PitchAngleControl((BasePID_Object *)(protract->Motors3508.turnPID + 0),protract->Protract_left.Target_Angle, protract->Protract_left.angle), protract->Motors3508.motor[0].Data.SpeedRPM);
	protract->Motors3508.motor[1].Data.Output = BasePID_YawSpeedControl((BasePID_Object *)(protract->Motors3508.turnPID + 3), BasePID_YawAngleControl((BasePID_Object *)(protract->Motors3508.turnPID + 2),protract->Protract_right.Target_Angle, protract->Protract_right.angle), protract->Motors3508.motor[1].Data.SpeedRPM);

	if (protract->Motors3508.motor[0].Data.Output >= 29000)
		protract->Motors3508.motor[0].Data.Output = 29000;
	if (protract->Motors3508.motor[0].Data.Output <= -29000)
		protract->Motors3508.motor[0].Data.Output = -29000;
	if (protract->Motors3508.motor[1].Data.Output >= 30000)
		protract->Motors3508.motor[1].Data.Output = 30000;
	if (protract->Motors3508.motor[1].Data.Output <= -30000)
		protract->Motors3508.motor[1].Data.Output = -30000;

	MotorFillData(&protract->Motors3508.motor[1], protract->Motors3508.motor[1].Data.Output);
	MotorFillData(&protract->Motors3508.motor[0], protract->Motors3508.motor[0].Data.Output);
}
void air_pressure_switch(RC_Ctrl *rc_ctrl)
{
   if(rc_ctrl->key_R_flag%2==1)
	 {
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	 }
	 if(rc_ctrl->key_R_flag%2==0)
	 {
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	 }
	 
	 if(rc_ctrl->key_SH_C_flag%2==1&&rc_ctrl->key_ctrl_flag!=1)
	 {
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
	 }
	 if(rc_ctrl->key_SH_C_flag%2==0&&rc_ctrl->key_ctrl_flag!=1)
	 {
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
	 }
	 
	 
	 if(rc_ctrl->key_B_flag%2==1)
	 {
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	 }
	 if(rc_ctrl->key_B_flag%2==0)
	 {
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	 }
	 
}
