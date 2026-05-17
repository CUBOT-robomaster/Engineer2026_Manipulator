#include <math.h>
#include "Custom_controller.h"
#include "math.h"
#include "arm_math.h"
#include "mecanum_chassis.h"
#include <driver_timer.h>
#define AtR 	0.017453277

float theta_1;
float theta_2;
float theta_3;

float l_active = 120;
float l_driven = 240;
float R        = 100;
float r        = 75;

//一堆向量
float OC1[3];
float OC2[3];
float OC3[3];
float C1B1[3];
float C2B2[3];
float C3B3[3];
float B1D1[3];
float B2D2[3];
float B3D3[3];
float OD1[3];
float OD2[3];
float OD3[3];
float D1D2[3];
float D1D3[3];
float D2D3[3];
float mid1[3];
float dir_FE[3];
float FE[3];
float OF[3];
float OE[3];
float dir_EP[3];
float EP[3];
float OP[3];

float a;
float b;
float c;
float p;
float S;

float norm_DiE;
float norm_FE;
float norm_EP;
float norm_mid1;
float norm_mid2;
/*------------------------------------------------------------*/
float l1=174.72,l2=170.38,l3;
float deg_yaw1;
float theta1 ;
float theta11;
float theta2 ;
float theta2_modify ;// 实际pitch2角度
float theta21 ;
float theta22 ;
float theta4,theta5;
float ds,dz,fs,fz;
/*-----------------------------------------------------------------*/
/**
*  @breif 求模长函数
*  @note  验证可用
*/
float Norm_cc(float array[3])
{
	float norm;
	norm = sqrt(pow(array[0],2) + pow(array[1],2) + pow(array[2],2));
	return norm;
}

/**
*  @breif 求叉积函数
*  @note  验证可用
*/
float *Cross(float arr1[3], float arr2[3]) {
	static float cross[3];
	cross[0] = arr1[1] * arr2[2] - arr1[2] * arr2[1];
	cross[1] = arr1[2] * arr2[0] - arr1[0] * arr2[2];
	cross[2] = arr1[0] * arr2[1] - arr1[1] * arr2[0];

	return cross;
}

/**
*  @breif 自定义控制器电机初始化
*  @note  初始化解算角度的三个电机
*/
void Controller_Init(Controller_s *Controller,BasePID_Object damp_pid)
{
	
	MotorInit(&Controller->Data_3508.motor[0],  0, Motor3508, CAN1 , 0x201);
	MotorInit(&Controller->Data_3508.motor[1],  0, Motor3508, CAN1 , 0x202);
	MotorInit(&Controller->Data_3508.motor[2],  0, Motor3508, CAN1 , 0x203);
	
	BasePID_Init(&Controller->Data_3508.turn_pid[0],damp_pid.Kp,damp_pid.Ki,damp_pid.Kd,damp_pid.KiPartDetachment);
	BasePID_Init(&Controller->Data_3508.turn_pid[1],damp_pid.Kp,damp_pid.Ki,damp_pid.Kd,damp_pid.KiPartDetachment);
	BasePID_Init(&Controller->Data_3508.turn_pid[2],damp_pid.Kp,damp_pid.Ki,damp_pid.Kd,damp_pid.KiPartDetachment);

}

	

void Controller_6020_Init_v1(Controller_s *Controller, BasePID_Object yaw_angle_pid ,BasePID_Object yaw_speed_pid, BasePID_Object yaw2_angle_pid ,BasePID_Object yaw2_speed_pid,BasePID_Object yaw3_angle_pid ,BasePID_Object yaw3_speed_pid, CanNumber canx)
{
		MotorInit(&Controller->Data_6020.motor[0],  203, Motor6020, canx , 0x207);
	    MotorInit(&Controller->Data_6020.motor[1],  393, Motor6020, canx , 0x206);
		MotorInit(&Controller->Data_6020.motor[2],  2379, Motor6020, canx , 0x205);
		BasePID_Init(&Controller->Data_6020.turn_pid[0], yaw_angle_pid.Kp, yaw_angle_pid.Ki, yaw_angle_pid.Kd, yaw_angle_pid.KiPartDetachment);
	    BasePID_Init(&Controller->Data_6020.turn_pid[1], yaw_speed_pid.Kp, yaw_speed_pid.Ki, yaw_speed_pid.Kd, yaw_speed_pid.KiPartDetachment);
	    BasePID_Init(&Controller->Data_6020.turn_pid[2], yaw2_angle_pid.Kp, yaw2_angle_pid.Ki, yaw2_angle_pid.Kd, yaw2_angle_pid.KiPartDetachment);
	    BasePID_Init(&Controller->Data_6020.turn_pid[3], yaw2_speed_pid.Kp, yaw2_speed_pid.Ki, yaw2_speed_pid.Kd, yaw2_speed_pid.KiPartDetachment);
		BasePID_Init(&Controller->Data_6020.turn_pid[4], yaw2_angle_pid.Kp, yaw2_angle_pid.Ki, yaw2_angle_pid.Kd, yaw2_angle_pid.KiPartDetachment);
	    BasePID_Init(&Controller->Data_6020.turn_pid[5], yaw3_speed_pid.Kp, yaw3_speed_pid.Ki, yaw3_speed_pid.Kd, yaw3_speed_pid.KiPartDetachment);


}
void Controller_3508_Init_v1(Controller_s *Controller, BasePID_Object yaw_angle_pid ,BasePID_Object yaw_speed_pid, BasePID_Object yaw2_angle_pid ,BasePID_Object yaw2_speed_pid,BasePID_Object yaw3_angle_pid ,BasePID_Object yaw3_speed_pid, CanNumber canx)
{
		MotorInit(&Controller->Data_3508.motor[0],  12366, Motor3508, canx , 0x201);
		MotorInit(&Controller->Data_3508.motor[1], 3188, Motor3508, canx , 0x202);
	    MotorInit(&Controller->Data_3508.motor[2],  81, Motor3508, canx , 0x203);
		BasePID_Init(&Controller->Data_3508.turn_pid[0], yaw_angle_pid.Kp, yaw_angle_pid.Ki, yaw_angle_pid.Kd, yaw_angle_pid.KiPartDetachment);
	    BasePID_Init(&Controller->Data_3508.turn_pid[1], yaw_speed_pid.Kp, yaw_speed_pid.Ki, yaw_speed_pid.Kd, yaw_speed_pid.KiPartDetachment);
	    BasePID_Init(&Controller->Data_3508.turn_pid[2], yaw2_angle_pid.Kp, yaw2_angle_pid.Ki, yaw2_angle_pid.Kd, yaw2_angle_pid.KiPartDetachment);
	    BasePID_Init(&Controller->Data_3508.turn_pid[3], yaw2_speed_pid.Kp, yaw2_speed_pid.Ki, yaw2_speed_pid.Kd, yaw2_speed_pid.KiPartDetachment);
	    BasePID_Init(&Controller->Data_3508.turn_pid[4], yaw3_angle_pid.Kp, yaw3_angle_pid.Ki, yaw3_angle_pid.Kd, yaw3_angle_pid.KiPartDetachment);
	    BasePID_Init(&Controller->Data_3508.turn_pid[5], yaw3_speed_pid.Kp, yaw3_speed_pid.Ki, yaw3_speed_pid.Kd, yaw3_speed_pid.KiPartDetachment);

}
/**
*  @breif 自定义控制器数据处理
*  @note  None
*/
void Controller_Dataprocs(Controller_s *Controller) {

	float *p1;
	float *p2;

	theta_1 = Controller->Data_3508.motor[0].Data.Angle - 84 + 36;
if(Controller->Data_3508.motor[1].Data.Angle >= 0)
	{
		theta_2 = Controller->Data_3508.motor[1].Data.Angle - 168;
	}
	else
	{
		theta_2 = 360 + Controller->Data_3508.motor[1].Data.Angle - 168;
	}
//	theta_2 = Controller->Data_3508.motor[1].Data.Angle;
	if(Controller->Data_3508.motor[2].Data.Angle >= 0)
	{
		theta_3 = Controller->Data_3508.motor[2].Data.Angle - 164+28;
	}
	else
	{
		theta_3 = 360 + Controller->Data_3508.motor[2].Data.Angle - 164+28;
	}

	/*-----第一部分：建立四棱锥模型-----*/
	//计算定平面的OCi向量
	OC1[0] = R;
	OC1[1] = 0;
	OC1[2] = 0;

	OC2[0] = -0.5 * R;
	OC2[1] = 0.5 * sqrt(3) * R;
	OC2[2] = 0;

	OC3[0] = -0.5 * R;
	OC3[1] = -0.5 * sqrt(3) * R;
	OC3[2] = 0;

	//计算主动臂的 CiBi OBi BiDi 向量
	C1B1[0] = l_active * cos(theta_1*AtR);
	C1B1[1] = 0;
	C1B1[2] = l_active * sin(theta_1*AtR);

	C2B2[0] = -0.5 * l_active * cos(theta_2*AtR);
	C2B2[1] = l_active * cos(theta_2*AtR) * 0.5 * sqrt(3);
	C2B2[2] = l_active * sin(theta_2*AtR);

	C3B3[0] = -0.5 * l_active * cos(theta_3*AtR);
	C3B3[1] = -l_active * cos(theta_3*AtR) * 0.5 * sqrt(3);
	C3B3[2] = l_active * sin(theta_3*AtR);

	B1D1[0] = -r;
	B1D1[1] = 0;
	B1D1[2] = 0;

	B2D2[0] = 0.5 * r;
	B2D2[1] = -r * sqrt(3) * 0.5;
	B2D2[2] = 0;

	B3D3[0] = 0.5 * r;
	B3D3[1] = r * sqrt(3) * 0.5;
	B3D3[2] = 0;

	//得到Di相对于定平面坐标系的坐标
	OD1[0] = OC1[0] + C1B1[0] + B1D1[0];
	OD1[1] = OC1[1] + C1B1[1] + B1D1[1];
	OD1[2] = OC1[2] + C1B1[2] + B1D1[2];

	OD2[0] = OC2[0] + C2B2[0] + B2D2[0];  //OC2没问题
	OD2[1] = OC2[1] + C2B2[1] + B2D2[1];
	OD2[2] = OC2[2] + C2B2[2] + B2D2[2];

	OD3[0] = OC3[0] + C3B3[0] + B3D3[0];
	OD3[1] = OC3[1] + C3B3[1] + B3D3[1];
	OD3[2] = OC3[2] + C3B3[2] + B3D3[2];

	/*-----第二部分：求解最终坐标-----*/
	//计算三角形D1D2D3的边长，使用norm获得向量的模长
	D1D2[0] = OD2[0] - OD1[0];
	D1D2[1] = OD2[1] - OD1[1];
	D1D2[2] = OD2[2] - OD1[2];

	D1D3[0] = OD3[0] - OD1[0];
	D1D3[1] = OD3[1] - OD1[1];
	D1D3[2] = OD3[2] - OD1[2];

	D2D3[0] = OD3[0] - OD2[0];
	D2D3[1] = OD3[1] - OD2[1];
	D2D3[2] = OD3[2] - OD2[2];

	a = Norm_cc(D1D2);
	b = Norm_cc(D1D3);
	c = Norm_cc(D2D3);
	p = 0.5 * (a + b + c);
	S = sqrt(p * (p - a) * (p - b) * (p - c));
	norm_DiE = a * b * c / (4 * S);
	norm_FE = sqrt(pow(norm_DiE, 2) - 0.25 * pow(a, 2));

	p1 = Cross(D1D2, D1D3);
	mid1[0] = *p1;
	mid1[1] = *(p1 + 1);
	mid1[2] = *(p1 + 2);

	p2 = Cross(mid1, D2D3);
	dir_FE[0] = *p2;
	dir_FE[1] = *(p2 + 1);
	dir_FE[2] = *(p2 + 2);
	norm_mid1 = Norm_cc(dir_FE);
	dir_FE[0] = dir_FE[0] / norm_mid1;
	dir_FE[1] = dir_FE[1] / norm_mid1;
	dir_FE[2] = dir_FE[2] / norm_mid1;

	FE[0] = dir_FE[0] * norm_FE;
	FE[1] = dir_FE[1] * norm_FE;
	FE[2] = dir_FE[2] * norm_FE;

	OF[0] = (OD2[0] + OD3[0]) * 0.5;
	OF[1] = (OD2[1] + OD3[1]) * 0.5;
	OF[2] = (OD2[2] + OD3[2]) * 0.5;
//------------------------------------------
	OE[0] = OF[0] + FE[0];
	OE[1] = OF[1] + FE[1];
	OE[2] = OF[2] + FE[2];

	D1D2[0] = -D1D2[0];
	D1D2[1] = -D1D2[1];
	D1D2[2] = -D1D2[2];
	p2 = Cross(D1D3, D1D2);
	dir_EP[0] = *p2 / (a * c);
	dir_EP[1] = *(p2 + 1) / (a * c);
	dir_EP[2] = *(p2 + 2) / (a * c);
	norm_mid2 = Norm_cc(dir_EP);
	dir_EP[0] = dir_EP[0] / norm_mid2;
	dir_EP[1] = dir_EP[1] / norm_mid2;
	dir_EP[2] = dir_EP[2] / norm_mid2;

	norm_EP = sqrt(pow(l_driven, 2) - pow(norm_DiE, 2));
	EP[0] = dir_EP[0] * norm_EP;
	EP[1] = dir_EP[1] * norm_EP;
	EP[2] = dir_EP[2] * norm_EP;

	Controller->DP.pos_x = -OE[2] - EP[2];
	Controller->DP.pos_y = OE[1] + EP[1];
	Controller->DP.pos_z = OE[0] + EP[0];

}
/**
*  @breif  计算控制器坐标移动速率
*  @note  
*/
float pos_xlast=0,pos_ylast=0,pos_zlast=0;
void speed_caculator(Controller_s *Controller)
{
	
	Controller->DP.pos_x_speed=(Controller->DP.pos_x-pos_xlast)*10;
	Controller->DP.pos_y_speed=(Controller->DP.pos_y-pos_ylast)*10;
	Controller->DP.pos_z_speed=(Controller->DP.pos_z-pos_zlast)*10;
	pos_xlast=Controller->DP.pos_x;
	pos_ylast=Controller->DP.pos_y;
	pos_zlast=Controller->DP.pos_z;
	
}
/**
*  @breif 自定义控制器电机初始化
*  @note  初始化解算角度的三个电机
*/
void Damp_test(Controller_s *Controller)
{
		
	Controller->Data_3508.motor[0].Data.Output = BasePID_SpeedControl((BasePID_Object*)(Controller->Data_3508.turn_pid+0),0,Controller->Data_3508.motor[0].Data.SpeedRPM);
	Controller->Data_3508.motor[1].Data.Output = BasePID_SpeedControl((BasePID_Object*)(Controller->Data_3508.turn_pid+1),0,Controller->Data_3508.motor[1].Data.SpeedRPM);
	Controller->Data_3508.motor[2].Data.Output = BasePID_SpeedControl((BasePID_Object*)(Controller->Data_3508.turn_pid+2),0,Controller->Data_3508.motor[2].Data.SpeedRPM);
	if(Controller->Data_3508.motor[0].Data.Output>=4000)
		Controller->Data_3508.motor[0].Data.Output=4000;
	if(Controller->Data_3508.motor[0].Data.Output<=-4000)
		Controller->Data_3508.motor[0].Data.Output=-4000;
	if(Controller->Data_3508.motor[1].Data.Output>=4000)
		Controller->Data_3508.motor[1].Data.Output=4000;
	if(Controller->Data_3508.motor[1].Data.Output<=-4000)
		Controller->Data_3508.motor[1].Data.Output=-4000;
	if(Controller->Data_3508.motor[2].Data.Output>=4000)
		Controller->Data_3508.motor[2].Data.Output=4000;
	if(Controller->Data_3508.motor[2].Data.Output<=-4000)
		Controller->Data_3508.motor[2].Data.Output=-4000;
	MotorFillData(&Controller->Data_3508.motor[2], Controller->Data_3508.motor[2].Data.Output);
	MotorFillData(&Controller->Data_3508.motor[1], Controller->Data_3508.motor[1].Data.Output);
	MotorFillData(&Controller->Data_3508.motor[0], Controller->Data_3508.motor[0].Data.Output);

		
}
float theta_one,theta_two,theta_3508,theta_two_last;
float Kp_1=12000,Kp_2=4000,compelete1,compelete2;
void Damp_test_v1(Controller_s *Controller)
{

		theta_one=theta1;
		theta_two=theta2;
		theta_3508=Controller->Data_3508.motor[1].Data.Angle+160;
		compelete1=-1*Kp_1*arm_sin_f32(deg_rad(theta1));
		compelete2=Kp_2*arm_sin_f32(deg_rad(theta2+90));
		for(int i=0;i<3;i++)
		{
			if((Controller->Data_6020.motor[i].Data.Angle<-100)&&(Controller->Data_6020.angle_last[i]>100))
		{
			Controller->Data_6020.angle[i] += 360 + Controller->Data_6020.motor[i].Data.Angle - Controller->Data_6020.angle_last[i];
		}
		else if(( Controller->Data_6020.motor[i].Data.Angle>100)&&(Controller->Data_6020.angle_last[i]<-100))
		{
			Controller->Data_6020.angle[i] += -360 + Controller->Data_6020.motor[i].Data.Angle - Controller->Data_6020.angle_last[i];
		}
		else 
		{
			Controller->Data_6020.angle[i] += Controller->Data_6020.motor[i].Data.Angle - Controller->Data_6020.angle_last[i];
		}
		Controller->Data_6020.angle_last[i] = Controller->Data_6020.motor[i].Data.Angle;
			if((Controller->Data_3508.motor[i].Data.Angle<-100)&&(Controller->Data_3508.angle_last[i]>100))
			{
				Controller->Data_3508.angle[i] += 360 + Controller->Data_3508.motor[i].Data.Angle - Controller->Data_3508.angle_last[i];
			}
			else if(( Controller->Data_3508.motor[i].Data.Angle>100)&&(Controller->Data_3508.angle_last[i]<-100))
			{
				Controller->Data_3508.angle[i] += -360 + Controller->Data_3508.motor[i].Data.Angle - Controller->Data_3508.angle_last[i];
			}
			else 
			{
				Controller->Data_3508.angle[i] += Controller->Data_3508.motor[i].Data.Angle - Controller->Data_3508.angle_last[i];
			}
			Controller->Data_3508.angle_last[i] = Controller->Data_3508.motor[i].Data.Angle;
		}
//		
		if(tim14.ClockTime%10==0)
		{
			Controller->Data_6020.target_angle[0] = Controller->Data_6020.deg_last[0];
			Controller->Data_6020.deg_last[0] = Controller->Data_6020.angle[0];
			Controller->Data_6020.target_angle[1] = Controller->Data_6020.deg_last[1];
			Controller->Data_6020.deg_last[1] = Controller->Data_6020.angle[1];
			Controller->Data_6020.target_angle[2] = Controller->Data_6020.deg_last[2];
			Controller->Data_6020.deg_last[2] = Controller->Data_6020.angle[2];
			
		}
		if(tim14.ClockTime%40==0)
		{
		Controller->Data_3508.target_angle[0] = Controller->Data_3508.deg_last[0];
			Controller->Data_3508.deg_last[0] = Controller->Data_3508.angle[0];
			Controller->Data_3508.target_angle[1] = Controller->Data_3508.deg_last[1];
			Controller->Data_3508.deg_last[1] = Controller->Data_3508.angle[1];
			Controller->Data_3508.target_angle[2] = Controller->Data_3508.deg_last[2];
			Controller->Data_3508.deg_last[2] = Controller->Data_3508.angle[2];
		}
		Controller->Data_6020.motor[0].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(Controller->Data_6020.turn_pid + 1) , BasePID_LiftingAngleControl((BasePID_Object*)(Controller->Data_6020.turn_pid + 0) , Controller->Data_6020.target_angle[0], Controller->Data_6020.angle[0])  ,Controller->Data_6020.motor[0].Data.SpeedRPM)+compelete1;
		Controller->Data_6020.motor[1].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(Controller->Data_6020.turn_pid + 3) , BasePID_LiftingAngleControl((BasePID_Object*)(Controller->Data_6020.turn_pid + 2) , Controller->Data_6020.target_angle[1], Controller->Data_6020.angle[1])  ,Controller->Data_6020.motor[1].Data.SpeedRPM)+compelete2;
		Controller->Data_6020.motor[2].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(Controller->Data_6020.turn_pid + 5) , BasePID_LiftingAngleControl((BasePID_Object*)(Controller->Data_6020.turn_pid + 4) , Controller->Data_6020.target_angle[2], Controller->Data_6020.angle[2])  ,Controller->Data_6020.motor[2].Data.SpeedRPM);
		Controller->Data_3508.motor[0].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(Controller->Data_3508.turn_pid + 1) , BasePID_LiftingAngleControl((BasePID_Object*)(Controller->Data_3508.turn_pid + 0) , Controller->Data_3508.target_angle[0], Controller->Data_3508.angle[0])  ,Controller->Data_3508.motor[0].Data.SpeedRPM);
		Controller->Data_3508.motor[1].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(Controller->Data_3508.turn_pid + 3) , BasePID_LiftingAngleControl((BasePID_Object*)(Controller->Data_3508.turn_pid + 2) , Controller->Data_3508.target_angle[1], Controller->Data_3508.angle[1])  ,Controller->Data_3508.motor[1].Data.SpeedRPM)*arm_sin_f32(deg_rad(theta_3508));
		Controller->Data_3508.motor[2].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(Controller->Data_3508.turn_pid + 5) , BasePID_LiftingAngleControl((BasePID_Object*)(Controller->Data_3508.turn_pid + 4) , Controller->Data_3508.target_angle[2], Controller->Data_3508.angle[2])  ,Controller->Data_3508.motor[2].Data.SpeedRPM);
		
		if (Controller->Data_6020.motor[0].Data.Output >= 24000)
			Controller->Data_6020.motor[0].Data.Output = 24000;
		if (Controller->Data_6020.motor[0].Data.Output <= -24000)
			Controller->Data_6020.motor[0].Data.Output = -24000;
		if (Controller->Data_6020.motor[1].Data.Output >= 24000)
			Controller->Data_6020.motor[1].Data.Output = 24000;
		if (Controller->Data_6020.motor[1].Data.Output <= -24000)
			Controller->Data_6020.motor[1].Data.Output = -24000;
		if (Controller->Data_6020.motor[2].Data.Output >= 24000)
			Controller->Data_6020.motor[2].Data.Output = 24000;
		if (Controller->Data_6020.motor[2].Data.Output <= -24000)
			Controller->Data_6020.motor[2].Data.Output = -24000;
		for(int i=0;i<3;i++)
		{
		if (Controller->Data_3508.motor[i].Data.Output >= 14000)
			Controller->Data_3508.motor[i].Data.Output = 14000;
		if (Controller->Data_3508.motor[i].Data.Output <= -14000)
			Controller->Data_3508.motor[i].Data.Output = -14000;
		}
		MotorFillData(&Controller->Data_6020.motor[0], Controller->Data_6020.motor[0].Data.Output);
		MotorFillData(&Controller->Data_6020.motor[1], Controller->Data_6020.motor[1].Data.Output);
		MotorFillData(&Controller->Data_6020.motor[2], Controller->Data_6020.motor[2].Data.Output);
		MotorFillData(&Controller->Data_3508.motor[0], Controller->Data_3508.motor[0].Data.Output);
		MotorFillData(&Controller->Data_3508.motor[1], Controller->Data_3508.motor[1].Data.Output);
		MotorFillData(&Controller->Data_3508.motor[2], Controller->Data_3508.motor[2].Data.Output);


}
	
/*% 机械臂正式测试位置信息正运动学解算单元
l1 = 174.72;
l2 = 170.38;
deg_yaw1 = 0;
theta1 = 0;         % 实际pitch1角度，参照仿真方向描述
theta11 = 90+theta1;
theta2 = 0;        % 实际pitch2角度
theta2 = -(90+theta2);
theta22 =18.48-theta2;
l3 = sqrt(l1^2 + l2^2 -2*l1*l2*cosd(theta22));

theta4 = acosd((l3^2+l1^2-l2^2)/(2*l1*l3));
theta5 = 180-(theta4 + theta11);
dz = l3 * sind(theta5);
ds = l3 * cosd(theta5);
fz = dz + 133;
fs = ds + 47;

x = -fs * cosd(deg_yaw1);
y = fs * sind(deg_yaw1);
z = fz;
*/
void uncoil_coordinate(Controller_s *Controller)
{
//	deg_yaw1=Controller->Data_6020.motor[2].Data.Angle; 
	        
	if(Controller->Data_6020.motor[0].Data.Angle>=0)
	theta1 = Controller->Data_6020.motor[0].Data.Angle-111; //实际pitch1角度，参照仿真方向描述	
	else if(Controller->Data_6020.motor[0].Data.Angle<0)
	theta1 = 360+Controller->Data_6020.motor[0].Data.Angle-111;
	theta2 = 1*(Controller->Data_6020.motor[1].Data.Angle+50);
	theta2_modify = theta2+(theta1); //实际pitch2角度
	deg_yaw1=0;
	Controller->DP.yaw=Controller->Data_6020.motor[2].Data.Angle;
	Controller->DP.pitch1=-1*theta1;
	Controller->DP.pitch2=-1*theta2_modify;
	Controller->DP.roll1=(Controller->Data_3508.motor[0].Data.Angle+9.6);
	Controller->DP.pitch3=1*(Controller->Data_3508.motor[1].Data.Angle+160-90);
	Controller->DP.roll2=Controller->Data_3508.angle[2];
//	theta1=0;
//	theta2=0;
//	theta11 = 90+theta1;
//	theta21 = -(90+theta2_modify);
//	theta22 =18.48-theta21;
//	l3 = sqrt(l1*l1 + l2*l2 -2*l1*l2*arm_cos_f32(deg_rad(theta22)));
//	theta4 = rad_deg(acos((l3*l3+l1*l1-l2*l2)/(2*l1*l3)));
//	theta5 = 180-(theta4 + theta11);
//	dz = l3 * arm_sin_f32(deg_rad(theta5));
//	ds = l3 * arm_cos_f32(deg_rad(theta5));
//	fz = dz + 133;
//	fs = ds + 47;
//	Controller->DP.pos_x = -fs * arm_cos_f32(deg_rad(deg_yaw1));
//	Controller->DP.pos_y = fs * arm_sin_f32(deg_rad(deg_yaw1));
//	Controller->DP.pos_z = fz;

}


//弧度转角度函数
float rad_deg(float rad)
{
	float deg;
	deg=rad*180/pi;
	return deg;

}
//角度度转弧度函数
float deg_rad(float deg)
{
	float rad;
	rad=deg*pi/180;
	return rad;

}