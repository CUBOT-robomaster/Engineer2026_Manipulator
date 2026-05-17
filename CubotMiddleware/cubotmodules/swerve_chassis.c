#include <swerve_chassis.h>
#include "hardware_config.h"
#define myABS(x) ( (x)>0?(x):-(x) )
#define AtR 0.0174532f	              //<  3.1415 /180 角度制 转化为弧度制	
int16_t targetspeed=0;

void VectorSolve(int16_t vx, int16_t vy, float* vectorAngle, int16_t* vectorModule, uint8_t id)
{
	float vxDivideVy;
	float angle;
	static float lastAngle[4];
	int16_t module;
	
	//< 计算接收机给出的期望平移速度合向量的角度和模值
	if((vx >100 || vx < -100) || (vy >100 || vy < -100))
	{
		vxDivideVy = (float)vx / (float)vy;
		angle =  57.29f * atan(vxDivideVy);
		module  =  sqrt(vx * vx + vy * vy);
		
		//< 将三角函数系下的角度值转换为0-180°和0-(-180°)
		if(vx >= 0 && vy < 0)
			angle = 180 + angle;
		else if(vx < 0 && vy < 0 )
			angle = -180 + angle;	
		if(id < 4)
			lastAngle[id] = angle;
	}
	else
	{
		angle = lastAngle[id];
		module  =  0;
	}
	if(id < 4)
		*vectorAngle  = angle;
	*vectorModule = module;
}


/**
  * @brief  将编码器数据特定的零点位置转换为+-180°的角度
  */
float EcdtoAngle(int16_t offset, int16_t ecd)
{
	float angle;
	if(offset < 4096)
	{
		if(ecd > offset + 4096)
			ecd = ecd - 8192;
	}
	else
	{
		if(ecd < offset - 4096)
			ecd = ecd + 8192;
	}
	angle = K_ECD_TO_ANGLE * (ecd - offset);
	return angle;
}


void SwerveChassisInit(SwerveChassis* chassis, BasePID_Object run_pid, BasePID_Object turn_pid, CanNumber canx)
{
    MotorInit(&chassis->Motors3508.motor[0], 0 , Motor3508, canx, 0x201);
	  MotorInit(&chassis->Motors3508.motor[1], 0 , Motor3508, canx, 0x202);
    MotorInit(&chassis->Motors3508.motor[2], 0 , Motor3508, canx, 0x203);
	  MotorInit(&chassis->Motors3508.motor[3], 0 , Motor3508, canx, 0x204);
 	
	  MotorInit(&chassis->Motors6020.motor[0], 3916, Motor6020, canx, 0x205);
	  MotorInit(&chassis->Motors6020.motor[1], 5170 , Motor6020, canx, 0x206);
	  MotorInit(&chassis->Motors6020.motor[2], 5136 , Motor6020, canx, 0x207);
	  MotorInit(&chassis->Motors6020.motor[3], 2372, Motor6020, canx, 0x208);
	
	  BasePID_Init(&chassis->Motors3508.RunPID[0], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	  BasePID_Init(&chassis->Motors3508.RunPID[1], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	  BasePID_Init(&chassis->Motors3508.RunPID[2], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	  BasePID_Init(&chassis->Motors3508.RunPID[3], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	
	  BasePID_Init(&chassis->Motors6020.TurnPID[0], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	  BasePID_Init(&chassis->Motors6020.TurnPID[1], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	  BasePID_Init(&chassis->Motors6020.TurnPID[2], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	  BasePID_Init(&chassis->Motors6020.TurnPID[3], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	
	 chassis->Movement.Sensitivity.Vx 	   = 5;
   chassis->Movement.Sensitivity.Vy      = 5;
   chassis->Movement.Sensitivity.Omega   = 4;
	 chassis->Movement.ModuleOfSpeed  	   = 0;  
	 chassis->Movement.AngleOfSpeed 	 	   = 0;		
    
}

float FindBestTargetAngle(float targetAngle, float feedbackAngle, uint8_t* flag)
{
	float vectorDeltaAngle;
	float deltaAngle;
	vectorDeltaAngle = targetAngle - feedbackAngle;
	
	if((vectorDeltaAngle)>180)
		vectorDeltaAngle = vectorDeltaAngle - 360.0f;
	else if ((vectorDeltaAngle)<-180)
		vectorDeltaAngle = 360 + vectorDeltaAngle;
		
	if(targetAngle >= 0.0f)
	{
		deltaAngle = vectorDeltaAngle - 180.0f;
		if((deltaAngle)>180)
			deltaAngle = deltaAngle - 360.0f;
		else if ((deltaAngle)<-180)
			deltaAngle = 360 + deltaAngle;
	}
	else if(targetAngle < 0.0f)
	{
		deltaAngle = vectorDeltaAngle + 180.0f;
		if((deltaAngle)>180)
			deltaAngle = deltaAngle - 360.0f;
		else if ((deltaAngle)<-180)
			deltaAngle = 360 + deltaAngle;
	}
	
	if(myABS(vectorDeltaAngle) > myABS(deltaAngle))
	{
		if(targetAngle >= 0.0f)
			targetAngle = targetAngle - 180.0f;
		else if(targetAngle < 0.0f)
			targetAngle = targetAngle + 180.0f;
		*flag = 1;
	}
	else 
	{
		*flag = 0;
	}
	return targetAngle;
}


void SwerveChassisGetRemoteData(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,float canAngle)
{
	{
		float angle = AtR * (-canAngle);
//(1024-660)*5=1832
		chassis->Movement.Vx     			 = -(rc_ctrl->rc.ch1 - 1024) * chassis->Movement.Sensitivity.Vx;	
		chassis->Movement.Vy		 		   = (rc_ctrl->rc.ch0 - 1024) * chassis->Movement.Sensitivity.Vy;	
		if(rc_ctrl->rc.sw<=700)
	  chassis->Movement.Omega   		 = 2000;
		else 
			chassis->Movement.Omega   		 = 0;
//		if(rc_ctrl->rc.sw >1400){
//		chassis->Movement.Omega 			 =rc_ctrl->rc.sw * 3; //-(rc_ctrl->rc.ch2 - 1024) * chassis->Movement.Sensitivity.Omega;
//		}
//		else chassis->Movement.Omega 			 = 0 ;
		

		int16_t rawVx = chassis->Movement.Vx;
		int16_t rawVy = chassis->Movement.Vy;
		chassis->Movement.Vx = (rawVx * cos(angle) - rawVy * sin(angle));
		chassis->Movement.Vy = (rawVx * sin(angle) + rawVy * cos(angle));
		
		VectorSolve(chassis->Movement.Vx, chassis->Movement.Vy, &chassis->Movement.AngleOfSpeed, &chassis->Movement.ModuleOfSpeed,8);
		
		//< 计算接收机给出的旋转分量与平移速度合向量在四个舵向电机上分解的角度
		chassis->Vectors.Vx[0] =  chassis->Movement.Vx + chassis->Movement.Omega;
		chassis->Vectors.Vy[0] =  chassis->Movement.Vy + chassis->Movement.Omega;
		//-------------------------------------------
		chassis->Vectors.Vx[1] =  chassis->Movement.Vx + chassis->Movement.Omega;
		chassis->Vectors.Vy[1] =  chassis->Movement.Vy - chassis->Movement.Omega;
		chassis->Vectors.Vx[2] =  chassis->Movement.Vx - chassis->Movement.Omega;
		chassis->Vectors.Vy[2] =  chassis->Movement.Vy - chassis->Movement.Omega;
		chassis->Vectors.Vx[3] =  chassis->Movement.Vx - chassis->Movement.Omega;
		chassis->Vectors.Vy[3] =  chassis->Movement.Vy + chassis->Movement.Omega;		
		
		//< 根据接收机水平和垂直和旋转的速度分量计算向量的模和夹角 
		VectorSolve(chassis->Vectors.Vx[0], chassis->Vectors.Vy[0], &chassis->Vectors.Angle[0] , &chassis->Vectors.SpeedNo[0],0);
		VectorSolve(chassis->Vectors.Vx[1], chassis->Vectors.Vy[1], &chassis->Vectors.Angle[1] , &chassis->Vectors.SpeedNo[1],1);
		VectorSolve(chassis->Vectors.Vx[2], chassis->Vectors.Vy[2], &chassis->Vectors.Angle[2] , &chassis->Vectors.SpeedNo[2],2);
		VectorSolve(chassis->Vectors.Vx[3], chassis->Vectors.Vy[3], &chassis->Vectors.Angle[3] , &chassis->Vectors.SpeedNo[3],3);
		
		
		//< 将遥控器计算的目标角度与当前反馈的电机角度对比，选择距离反馈角度最近的目标角度，并且判断电机正反转是否需要改变
		chassis->Vectors.BestAngle[0] = FindBestTargetAngle(chassis->Vectors.Angle[0], chassis->Motors6020.motor[0].Data.Angle, &chassis->Vectors.SpeedChangeFlag[0]);
		chassis->Vectors.BestAngle[1] = FindBestTargetAngle(chassis->Vectors.Angle[1], chassis->Motors6020.motor[1].Data.Angle, &chassis->Vectors.SpeedChangeFlag[1]);
		chassis->Vectors.BestAngle[2] = FindBestTargetAngle(chassis->Vectors.Angle[2], chassis->Motors6020.motor[2].Data.Angle, &chassis->Vectors.SpeedChangeFlag[2]);
		chassis->Vectors.BestAngle[3] = FindBestTargetAngle(chassis->Vectors.Angle[3], chassis->Motors6020.motor[3].Data.Angle, &chassis->Vectors.SpeedChangeFlag[3]);
		
		chassis->Vectors.TargetEcd[0] = chassis->Motors6020.motor[0].Param.EcdOffset + 8192.0f*((float)chassis->Vectors.BestAngle[0]/(float)360.0f);
		chassis->Vectors.TargetEcd[1] = chassis->Motors6020.motor[1].Param.EcdOffset + 8192.0f*((float)chassis->Vectors.BestAngle[1]/(float)360.0f);
		chassis->Vectors.TargetEcd[2] = chassis->Motors6020.motor[2].Param.EcdOffset + 8192.0f*((float)chassis->Vectors.BestAngle[2]/(float)360.0f);
		chassis->Vectors.TargetEcd[3] = chassis->Motors6020.motor[3].Param.EcdOffset + 8192.0f*((float)chassis->Vectors.BestAngle[3]/(float)360.0f);
		
				 
		//< 以目标角度的编码器值为零点，计算电机当前反馈角度编码器值对应的角度 (-180 <- 0 -> 180)
		chassis->Vectors.FeedbackAngle[0] = EcdtoAngle(chassis->Vectors.TargetEcd[0], chassis->Motors6020.motor[0].Data.RawEcd);
		chassis->Vectors.FeedbackAngle[1] = EcdtoAngle(chassis->Vectors.TargetEcd[1], chassis->Motors6020.motor[1].Data.RawEcd);
		chassis->Vectors.FeedbackAngle[2] = EcdtoAngle(chassis->Vectors.TargetEcd[2], chassis->Motors6020.motor[2].Data.RawEcd);
		chassis->Vectors.FeedbackAngle[3] = EcdtoAngle(chassis->Vectors.TargetEcd[3], chassis->Motors6020.motor[3].Data.RawEcd);
	}
}
void SwerveChassisMotionControl(SwerveChassis* chassis)
{
	
	for(int j =0; j<4; j++)
	{
		float deltaAngle = chassis->Vectors.BestAngle[j] -chassis->Vectors.Angle[j];

		if(chassis->Vectors.SpeedChangeFlag[j] == 1)
			chassis->Vectors.SpeedNo[j] = - chassis->Vectors.SpeedNo[j];
		else
			chassis->Vectors.SpeedNo[j] = chassis->Vectors.SpeedNo[j];
		if(j == 1)
			chassis->Motors3508.motor[j].Data.Target = chassis->Vectors.SpeedNo[j];
		else if(j == 2)
			chassis->Motors3508.motor[j].Data.Target = chassis->Vectors.SpeedNo[j];
		else
			chassis->Motors3508.motor[j].Data.Target = -chassis->Vectors.SpeedNo[j];
	}
	
	
	for(int i=0;i<4;i++)  //< 计算底盘电机
	{   
		if( chassis->Motors3508.motor[i].Data.Target >  7800)   chassis->Motors3508.motor[i].Data.Target =  7800;
		if( chassis->Motors3508.motor[i].Data.Target < -7800)   chassis->Motors3508.motor[i].Data.Target = -7800;
		chassis->Motors3508.motor[i].Data.Output = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors3508.RunPID + i), chassis->Motors3508.motor[i].Data.Target, chassis->Motors3508.motor[i].Data.SpeedRPM);
	}
	
	  chassis->Motors6020.motor[0].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 0), -chassis->Vectors.FeedbackAngle[0], 0, 0.1047f*chassis->Motors6020.motor[0].Data.SpeedRPM); //< RPM转换为rad/s 单位转换系数
	  chassis->Motors6020.motor[1].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 1), -chassis->Vectors.FeedbackAngle[1], 0, 0.1047f*chassis->Motors6020.motor[1].Data.SpeedRPM);
	  chassis->Motors6020.motor[2].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 2), -chassis->Vectors.FeedbackAngle[2], 0, 0.1047f*chassis->Motors6020.motor[2].Data.SpeedRPM);
	  chassis->Motors6020.motor[3].Data.Output = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + 3), -chassis->Vectors.FeedbackAngle[3], 0, 0.1047f*chassis->Motors6020.motor[3].Data.SpeedRPM);
	
	
	
	    for(int i=0;i<4;i++) {
			MotorFillData(&chassis->Motors3508.motor[i], chassis->Motors3508.motor[i].Data.Output);
		}
    	MotorFillData(&chassis->Motors6020.motor[0],chassis->Motors6020.motor[0].Data.Output);
	    MotorFillData(&chassis->Motors6020.motor[1],chassis->Motors6020.motor[1].Data.Output);	
    	MotorFillData(&chassis->Motors6020.motor[2],chassis->Motors6020.motor[2].Data.Output);
    	MotorFillData(&chassis->Motors6020.motor[3],chassis->Motors6020.motor[3].Data.Output);
//		chassis->Motors3508.motor[0].Data.Output = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors3508.RunPID), targetspeed, chassis->Motors3508.motor[0].Data.SpeedRPM);
//   chassis->Power.speedfinalout[0]= chassis->Motors3508.motor[0].Data.Output;

//	 MotorFillData(&chassis->Motors3508.motor[0],chassis->Motors3508.motor[0].Data.Output);
//   MotorCanOutput(can2, 0x200);

//	MotorFillData(&chassis->Motors3508.motor[0],0);
//   MotorCanOutput(can2, 0x200);

}

void SwerveChassisOutputControl(SwerveChassis* chassis, RC_Ctrl rcCtrl)
{
	if(rcCtrl.isOnline == 1) 
	{
		MotorCanOutput(can2, 0x1ff);  
		MotorCanOutput(can2, 0x200);		
	}
	else 
	{ //< 接收机离线，电机输出为0
		MotorFillData(&chassis->Motors6020.motor[0],0);
		MotorFillData(&chassis->Motors6020.motor[1],0);
		MotorFillData(&chassis->Motors6020.motor[2],0);
		MotorFillData(&chassis->Motors6020.motor[3],0);
		MotorFillData(&chassis->Motors3508.motor[0],0);
		MotorFillData(&chassis->Motors3508.motor[1],0);
		MotorFillData(&chassis->Motors3508.motor[2],0);
		MotorFillData(&chassis->Motors3508.motor[3],0);
		MotorCanOutput(can2, 0x1ff);
		MotorCanOutput(can2, 0x200);
	}
}

