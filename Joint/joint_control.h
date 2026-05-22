#ifndef __JOINT_CONTROL_H_
#define __JOINT_CONTROL_H_
#include "motor.h"
#include "pid.h"
#include "servo.h"
#include "interaction_image.h"
#define JOINT_VELOCITY_SENSITIVITY 1.2f

extern float test_angle;

extern float open_angle_right;
extern float open_angle_left;

typedef struct{
	uint8_t CAN_ID;
	uint8_t id;
	uint8_t Error;
	float pos;
	float vel;
	float T;
	int8_t T_mos;//驱动mos温度
	int8_t T_Rotor;//电机线圈温度
	float PMAX;
}DM_motor;

struct Manipulator_motor_information{
    float Target_Angle;
    float Can_Angle;
    float Last_Can_Angle;
    float Can_Angle_speed;
    float angle;
    float angle_last;
    int16_t Can_Speed_Feedback;
};
typedef struct
{
 struct Manipulator_motor_information roll1;
 Motor motor6020;
 BasePID_Object turnpid[4];
 
 
 }motor6020_t;
typedef struct
{
	
	float rad;
	float rad_init;
	float rad_target;
	float angle;
	float angle_add;
	float angle_init;
	float angle_target;
	float velocity;				//速度
	float cc_init;				//自定义控制器初始化角度
	float cc_recv;				//自定义控制器当前角度
	float max_velocity;			//最小速度
	float min_velocity;			//最小速度
	float sensitivity;			//控制各关节运动的灵敏度
	float zero_point;			//准备开启或关闭自定义控制器模式时进行复位的电机位置
	float limit_min;			//限幅最小值
	float limit_max;			//限幅最大值
	float land_point;			//登岛复位位置
	float step_out_point;		//步进复位位置
	uint8_t init_flag;			//角度初始化标志位

	float Accel;				//加速度
	float Accel_sensitivity;	//加速度灵敏度
    
}joint_angle_t;



typedef struct{
	DM_motor  Dm_4340_joint1;
	DM_motor  Dm_4340_joint2;
	DM_motor  Dm_8006_joint3;
	DM_motor  Dm_4310_joint4;
	DM_motor  Dm_4310_joint5;
	DM_motor  Dm_4310_joint6;
	
	joint_angle_t joint0_deg;
	joint_angle_t joint1_deg;
	joint_angle_t joint2_deg;
	joint_angle_t joint3_deg;
	joint_angle_t joint4_deg;
	joint_angle_t joint5_deg;
	joint_angle_t joint6_deg;
	
	uint8_t controller_mapping_flag;	//自定义控制器映射标志位
	uint16_t controller_mapping_count;  
	
	Feetech_servo clamp_jaw;			//夹爪两个舵机的数据
}Manipulator_t;

extern Manipulator_t Manipulator_Right;
extern Manipulator_t Manipulator_Left;

void Motor_8010_Init(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left);
void Manipulator_angle_check(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left);
void joints_data_filter(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, custom_robot_data_t* custom);

void joint0_Ctrl_right(Manipulator_t* manipulator);
void joint1_Ctrl_right(Manipulator_t* manipulator);
void joint2_Ctrl_right(Manipulator_t* manipulator);
void joint3_Ctrl_right(Manipulator_t* manipulator);
void joint4_Ctrl_right(Manipulator_t* manipulator);
void joint5_Ctrl_right(Manipulator_t* manipulator);

void joint6_Ctrl_right(Manipulator_t* manipulator);
void joint0_Ctrl_left(Manipulator_t* manipulator);
void joint1_Ctrl_left(Manipulator_t* manipulator);
void joint2_Ctrl_left(Manipulator_t* manipulator);
void joint3_Ctrl_left(Manipulator_t* manipulator);
void joint4_Ctrl_left(Manipulator_t* manipulator);
void joint5_Ctrl_left(Manipulator_t* manipulator);
void joint6_Ctrl_left(Manipulator_t* manipulator);

float joints_limit(float target_angle,float min_angle,float max_angle);
void Clamp_jaw_angle_test(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left);
void joint_Ctrl(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left);
void all_joints_limit(Manipulator_t* manipulator);
#endif