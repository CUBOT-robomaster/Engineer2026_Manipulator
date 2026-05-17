#include "joint_control.h"
#include "mit.h"
#include "dr16.h"
#include "motor.h"
#include "driver_timer.h"
#include "AIM_PDO.h"
#include "interaction_image.h"
#include "controllerl.h"
#include "filter.h"
#include "velocity_pl.h"
#include "servo.h"
#include "auto.h"

/**
  * @brief 关节控制模块
  */

uint8_t init_angle_check_flag = 0;  		//角度正确标志位

int8_t motor_8010_flag = 1;  				//joint0初始化标志位
int16_t motor_8010_count = 0;  				//joint0初始化计数

float test_angle0 = 0;
float test_angle1 = 0;
float test_angle2 = 0;
float test_angle3 = 0;

float open_angle_right = 0;
float open_angle_left = 0;

float sensitivity = 1.5;

/* 初始化电机数据 */
Manipulator_t Manipulator_Right = {
	/* 初始化电机ID */
	.Dm_4340_joint2.CAN_ID = 0x01,
	.Dm_4340_joint1.CAN_ID = 0x02,
	.Dm_8006_joint3.CAN_ID = 0x03,
	.Dm_4310_joint4.CAN_ID = 0x04,
	.Dm_4310_joint5.CAN_ID = 0x05,
	.Dm_4310_joint6.CAN_ID = 0x06,

	/* 初始化目标角度 */
	.joint1_deg.angle_target = 0,
	.joint2_deg.angle_target = 0,
	.joint3_deg.angle_target = 0,
	.joint4_deg.angle_target = 0,
	.joint5_deg.angle_target = 0,
	.joint6_deg.angle_target = 0,

	/* 初始化运动量 */
	.joint0_deg.velocity = 0,
	.joint0_deg.max_velocity = 0.012 * JOINT_VELOCITY_SENSITIVITY,
	.joint0_deg.min_velocity = 0.06 * JOINT_VELOCITY_SENSITIVITY,
	.joint0_deg.Accel = 0.0032 * JOINT_VELOCITY_SENSITIVITY,
	.joint0_deg.sensitivity = 2.5,
	.joint0_deg.zero_point = 0.462,			//弧度制,对应角度值为26.4
	.joint0_deg.limit_min = -68,			//角度制
	.joint0_deg.limit_max = 114,
	.joint0_deg.land_point = -0.21,			//弧度制，对应角度制为-12
	.joint0_deg.step_out_point = -0.04,		//弧度制

	.joint1_deg.velocity = 0,
	.joint1_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint1_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint1_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint1_deg.sensitivity = 1.8,
	.joint1_deg.zero_point = -2.8,
	.joint1_deg.limit_min = -3,
	.joint1_deg.limit_max = -1,
	.joint1_deg.land_point = -2.7,
	.joint1_deg.step_out_point = -2.7,

	.joint2_deg.velocity = 0,
	.joint2_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint2_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint2_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint2_deg.sensitivity = 1.75,
	.joint2_deg.zero_point = -0.5,
	.joint2_deg.limit_min = -1.5,
	.joint2_deg.limit_max = 2,
	.joint2_deg.land_point = -0.5,
	.joint2_deg.step_out_point = -0.5,

	.joint3_deg.velocity = 0,
	.joint3_deg.max_velocity = 0.18 * JOINT_VELOCITY_SENSITIVITY,
	.joint3_deg.min_velocity = 0.06 * JOINT_VELOCITY_SENSITIVITY,
	.joint3_deg.Accel = 0.00024 * JOINT_VELOCITY_SENSITIVITY,
	.joint3_deg.sensitivity = 1.4,
	.joint3_deg.zero_point = -1.55,
	.joint3_deg.limit_min = -2.6,
	.joint3_deg.limit_max = 0.5,
	.joint3_deg.land_point = -2,
	.joint3_deg.step_out_point = -2.57,

	.joint4_deg.velocity = 0,
	.joint4_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint4_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint4_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint4_deg.sensitivity = 1.0,
	.joint4_deg.zero_point = -0.03,
	.joint4_deg.limit_min = -3,
	.joint4_deg.limit_max = 3,
	.joint4_deg.land_point = 0.17,
	.joint4_deg.step_out_point = 0.17,

	.joint5_deg.velocity = 0,
	.joint5_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint5_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint5_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint5_deg.sensitivity = 1.6,
	.joint5_deg.zero_point = -0.01,
	.joint5_deg.limit_min = -1.36,
	.joint5_deg.limit_max = 1.84,
	.joint5_deg.land_point = -0.01,
	.joint5_deg.step_out_point = -0.01,

	.joint6_deg.velocity = 0,
	.joint6_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint6_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint6_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint6_deg.sensitivity = 1.0,
	.joint6_deg.zero_point = -0.04,
	.joint6_deg.limit_min = -3,
	.joint6_deg.limit_max = 3,
	.joint6_deg.land_point = -0.04,
	.joint6_deg.step_out_point = -0.04,

	.clamp_jaw.STS_3215[0].servo_id = 1,									//position增大张开
	.clamp_jaw.STS_3215[0].position = SERVO_RIGHT_ZERO_OPEN_ID_ONE,			//1550开,1350关
	.clamp_jaw.STS_3215[0].velocity = 2000,

	.clamp_jaw.STS_3215[1].servo_id = 3,									//position减小张开
	.clamp_jaw.STS_3215[1].position = SERVO_RIGHT_ONE_OPEN_ID_THREE,		//1450开，1650关
	.clamp_jaw.STS_3215[1].velocity = 2000,
};

Manipulator_t Manipulator_Left = {
	/* 初始化电机ID */
	.Dm_4340_joint1.CAN_ID = 0x01,
	.Dm_4340_joint2.CAN_ID = 0x02,
	.Dm_8006_joint3.CAN_ID = 0x03,
	.Dm_4310_joint4.CAN_ID = 0x05,
	.Dm_4310_joint5.CAN_ID = 0x04,
	.Dm_4310_joint6.CAN_ID = 0x06,

	/* 初始化目标角度 */
	.joint1_deg.angle_target = 0,
	.joint2_deg.angle_target = 0,
	.joint3_deg.angle_target = 0,
	.joint4_deg.angle_target = 0,
	.joint5_deg.angle_target = 0,
	.joint6_deg.angle_target = 0,

	/* 初始化运动量 */
	.joint0_deg.velocity = 0,
	.joint0_deg.max_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint0_deg.min_velocity = 0.06 * JOINT_VELOCITY_SENSITIVITY,
	.joint0_deg.Accel = 0.0032 * JOINT_VELOCITY_SENSITIVITY,
	.joint0_deg.sensitivity = 1.0,
	.joint0_deg.zero_point = 3.72,			//弧度制，对应角度制为213
	.joint0_deg.limit_min = 96,				//角度制
	.joint0_deg.limit_max = 296,
	.joint0_deg.land_point = 4.5,			//弧度制，对应角度制为258
	.joint0_deg.step_out_point = 4.18,		//弧度制

	.joint1_deg.velocity = 0,
	.joint1_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint1_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint1_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint1_deg.sensitivity = 1.0,
	.joint1_deg.zero_point = 0.9,
	.joint1_deg.limit_min = -0.9,
	.joint1_deg.limit_max = 0.95,
	.joint1_deg.land_point = 0.77,
	.joint1_deg.step_out_point = 0.77,

	.joint2_deg.velocity = 0,
	.joint2_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint2_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint2_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint2_deg.sensitivity = 1.0,
	.joint2_deg.zero_point = 0.36,
	.joint2_deg.limit_min = -1.9,
	.joint2_deg.limit_max = 1.7,
	.joint2_deg.land_point = 0.36,
	.joint2_deg.step_out_point = 0.36,

	.joint3_deg.velocity = 0,
	.joint3_deg.max_velocity = 0.18 * JOINT_VELOCITY_SENSITIVITY,
	.joint3_deg.min_velocity = 0.06 * JOINT_VELOCITY_SENSITIVITY,
	.joint3_deg.Accel = 0.00024 * JOINT_VELOCITY_SENSITIVITY,
	.joint3_deg.sensitivity = 1.0,
	.joint3_deg.zero_point = -0.5,
	.joint3_deg.limit_min = -1.6,
	.joint3_deg.limit_max = 1.8,
	.joint3_deg.land_point = -0.88,
	.joint3_deg.step_out_point = -1.58,

	.joint4_deg.velocity = 0,
	.joint4_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint4_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint4_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint4_deg.sensitivity = 1.0,
	.joint4_deg.zero_point = -0.03,
	.joint4_deg.limit_min = -3.43,
	.joint4_deg.limit_max = 3.37,
	.joint4_deg.land_point = -0.18,
	.joint4_deg.step_out_point = -0.18,

	.joint5_deg.velocity = 0,
	.joint5_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint5_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint5_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint5_deg.sensitivity = 1.0,
	.joint5_deg.zero_point = -0.06,
	.joint5_deg.limit_min = -1.8,
	.joint5_deg.limit_max = 1.5,
	.joint5_deg.land_point = -0.06,
	.joint5_deg.step_out_point = -0.06,

	.joint6_deg.velocity = 0,
	.joint6_deg.max_velocity = 0.32 * JOINT_VELOCITY_SENSITIVITY,
	.joint6_deg.min_velocity = 0.12 * JOINT_VELOCITY_SENSITIVITY,
	.joint6_deg.Accel = 0.006 * JOINT_VELOCITY_SENSITIVITY,
	.joint6_deg.sensitivity = 1.0,
	.joint6_deg.zero_point = -0.16,
	.joint6_deg.limit_min = -3.56,
	.joint6_deg.limit_max = 3.24,
	.joint6_deg.land_point = -0.16,
	.joint6_deg.step_out_point = -0.16,

	.clamp_jaw.STS_3215[0].servo_id = 0,			//position减小张开
	.clamp_jaw.STS_3215[0].position = SERVO_LEFT_ZERO_OPEN_ID_ZERO,	//1425开，1625关
	.clamp_jaw.STS_3215[0].velocity = 2000,

	.clamp_jaw.STS_3215[1].servo_id = 2,			//position增大张开
	.clamp_jaw.STS_3215[1].position = SERVO_LEFT_ONE_OPEN_ID_TWO,	//1500开，1300关
	.clamp_jaw.STS_3215[1].velocity = 2000,
};

extern struct joint_deg Joint_deg;
int inverse_count = 0;

/**
* @brief 关节限制函数。防止堵转
  */ 
float joints_limit(float target_angle,float min_angle,float max_angle)
{
	if(target_angle > max_angle)
		target_angle = max_angle;
	if(target_angle < min_angle)
		target_angle = min_angle;
	return target_angle;
}

void all_joints_limit(Manipulator_t* manipulator){
	manipulator -> joint0_deg.angle = joints_limit(manipulator -> joint0_deg.angle, manipulator -> joint0_deg.limit_min, manipulator -> joint0_deg.limit_max);
	manipulator -> joint1_deg.rad = joints_limit(manipulator -> joint1_deg.rad, manipulator -> joint1_deg.limit_min, manipulator -> joint1_deg.limit_max);
	manipulator -> joint2_deg.rad = joints_limit(manipulator -> joint2_deg.rad, manipulator -> joint2_deg.limit_min, manipulator -> joint2_deg.limit_max);
	manipulator -> joint3_deg.rad = joints_limit(manipulator -> joint3_deg.rad, manipulator -> joint3_deg.limit_min, manipulator -> joint3_deg.limit_max);
	manipulator -> joint4_deg.rad = joints_limit(manipulator -> joint4_deg.rad, manipulator -> joint4_deg.limit_min, manipulator -> joint4_deg.limit_max);
	manipulator -> joint5_deg.rad = joints_limit(manipulator -> joint5_deg.rad, manipulator -> joint5_deg.limit_min, manipulator -> joint5_deg.limit_max);
	manipulator -> joint6_deg.rad = joints_limit(manipulator -> joint6_deg.rad, manipulator -> joint6_deg.limit_min, manipulator -> joint6_deg.limit_max);
}

void Motor_8010_Init(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left){
	if(motor_8010_count < 900 && motor_8010_flag == 1){
		if(manipulator_right -> joint0_deg.init_flag == 0){
			RPDO2_pos(&can1,0);
			manipulator_right -> joint0_deg.angle_init = MOTOR_ONE.pos;
			manipulator_right -> joint0_deg.angle = manipulator_right -> joint0_deg.angle_init;
			manipulator_right -> joint0_deg.rad_init = manipulator_right -> joint0_deg.angle_init * AtR;
		}
		
		if(manipulator_left -> joint0_deg.init_flag == 0){
			RPDO2_pos(&can2,0);
			manipulator_left -> joint0_deg.angle_init = MOTOR_TWO.pos;
			manipulator_left -> joint0_deg.angle = manipulator_left -> joint0_deg.angle_init;
			manipulator_left -> joint0_deg.rad_init = manipulator_left -> joint0_deg.angle_init * AtR;
		}
	}
	if(motor_8010_count == 900){
		motor_8010_flag = 0;
		manipulator_right -> joint0_deg.init_flag = 1;
		manipulator_left -> joint0_deg.init_flag = 1;
	}
}
	
void Manipulator_angle_check(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left){
	/* 初始角度检查 */
	if((manipulator_right -> joint4_deg.rad_init > -1 && manipulator_right -> joint4_deg.rad_init < 1) && (manipulator_right -> joint6_deg.rad_init > -1 && manipulator_right -> joint6_deg.rad_init < 1)){
		if(manipulator_left -> joint4_deg.rad_init > -1 && manipulator_left -> joint4_deg.rad_init < 1 && manipulator_left -> joint6_deg.rad_init > -1 && manipulator_left -> joint6_deg.rad_init < 1){
			init_angle_check_flag = 1;
		}
		else{
			init_angle_check_flag = 0;
		}
	}
}

void joint0_Ctrl_right(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch2 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint0_deg.angle_target -= -0.03;
//	else if(rc_Ctrl.rc.ch2 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint0_deg.angle_target += -0.03;

	if(manipulator -> joint0_deg.angle_target + manipulator -> joint0_deg.angle_init > manipulator -> joint0_deg.angle + 0.01)
		manipulator -> joint0_deg.angle += velocity_plan(&manipulator -> joint0_deg);
	else if(manipulator -> joint0_deg.angle_target + manipulator -> joint0_deg.angle_init < manipulator -> joint0_deg.angle - 0.01)
		manipulator -> joint0_deg.angle -= velocity_plan(&manipulator -> joint0_deg); 
	manipulator -> joint0_deg.rad = manipulator -> joint0_deg.angle * AtR;
}

void joint1_Ctrl_right(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch3 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint1_deg.angle_target += 0.01*sensitivity;
//	else if(rc_Ctrl.rc.ch3 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint1_deg.angle_target -= 0.01*sensitivity;
 
	if(manipulator -> joint1_deg.angle_target + manipulator -> joint1_deg.angle_init > manipulator -> joint1_deg.angle + 0.01)
		manipulator -> joint1_deg.angle += velocity_plan(&manipulator -> joint1_deg);
	else if(manipulator -> joint1_deg.angle_target + manipulator -> joint1_deg.angle_init < manipulator -> joint1_deg.angle - 0.01)
		manipulator -> joint1_deg.angle -= velocity_plan(&manipulator -> joint1_deg); 
	manipulator -> joint1_deg.rad = manipulator -> joint1_deg.angle * AtR;
}

void joint2_Ctrl_right(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch0 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint2_deg.angle_target += 0.01;
//	else if(rc_Ctrl.rc.ch0 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint2_deg.angle_target -= 0.01;
	
	if(manipulator -> joint2_deg.angle_target + manipulator -> joint2_deg.angle_init > manipulator -> joint2_deg.angle + 0.01)
		manipulator -> joint2_deg.angle += velocity_plan(&manipulator -> joint2_deg);
	else if(manipulator -> joint2_deg.angle_target + manipulator -> joint2_deg.angle_init < manipulator -> joint2_deg.angle - 0.01)
		manipulator -> joint2_deg.angle -= velocity_plan(&manipulator -> joint2_deg); 
	manipulator -> joint2_deg.rad = manipulator -> joint2_deg.angle * AtR;
}

void joint3_Ctrl_right(Manipulator_t* manipulator){
	// if(rc_Ctrl.rc.ch1 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
	// 	manipulator -> joint3_deg.angle_target += -0.01;
	// else if(rc_Ctrl.rc.ch1 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
	// 	manipulator -> joint3_deg.angle_target -= -0.01;

	if(manipulator -> joint3_deg.angle_target + manipulator -> joint3_deg.angle_init > manipulator -> joint3_deg.angle + 0.01)
		manipulator -> joint3_deg.angle += velocity_plan(&manipulator -> joint3_deg);
	else if(manipulator -> joint3_deg.angle_target + manipulator -> joint3_deg.angle_init < manipulator -> joint3_deg.angle - 0.01)
		manipulator -> joint3_deg.angle -= velocity_plan(&manipulator -> joint3_deg); 
	manipulator -> joint3_deg.rad = manipulator -> joint3_deg.angle * AtR;
}



void joint4_Ctrl_right(Manipulator_t* manipulator){
		/* 遥控模式 */
//	if(rc_Ctrl.rc.ch2 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint4_deg.angle_target += 0.01;
//	else if(rc_Ctrl.rc.ch2 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint4_deg.angle_target -= 0.01;

	if(manipulator -> joint4_deg.angle_target + manipulator -> joint4_deg.angle_init > manipulator -> joint4_deg.angle + 0.01)
		manipulator -> joint4_deg.angle += velocity_plan(&manipulator -> joint4_deg);
	else if(manipulator -> joint4_deg.angle_target + manipulator -> joint4_deg.angle_init < manipulator -> joint4_deg.angle - 0.01)
		manipulator -> joint4_deg.angle -= velocity_plan(&manipulator -> joint4_deg); 
	manipulator -> joint4_deg.rad = manipulator -> joint4_deg.angle * AtR;
}

void joint5_Ctrl_right(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch3 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint5_deg.angle_target -= 0.01;
//	else if(rc_Ctrl.rc.ch3 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint5_deg.angle_target += 0.01;
	
	if(manipulator -> joint5_deg.angle_target + manipulator -> joint5_deg.angle_init > manipulator -> joint5_deg.angle + 0.01)
		manipulator -> joint5_deg.angle += velocity_plan(&manipulator -> joint5_deg);
	else if(manipulator -> joint5_deg.angle_target + manipulator -> joint5_deg.angle_init < manipulator -> joint5_deg.angle - 0.01)
		manipulator -> joint5_deg.angle -= velocity_plan(&manipulator -> joint5_deg); 
	manipulator -> joint5_deg.rad = manipulator -> joint5_deg.angle * AtR;
}

void joint6_Ctrl_right(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch0 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint6_deg.angle_target += 0.01;
//	else if(rc_Ctrl.rc.ch0 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint6_deg.angle_target -= 0.01;
	
	/* 自定义控制器模式 */
	if(Custom.image_recv.Coordinate.right_index_switch == 1 && manipulator -> controller_mapping_flag % 2 == 1)
		manipulator -> joint6_deg.angle_target -= 0.05 * manipulator -> joint6_deg.sensitivity;
	else if(Custom.image_recv.Coordinate.right_index_switch == -1 && manipulator -> controller_mapping_flag % 2 == 1)
		manipulator -> joint6_deg.angle_target += 0.05 * manipulator -> joint6_deg.sensitivity;
	
	if(manipulator -> joint6_deg.angle_target + manipulator -> joint6_deg.angle_init > manipulator -> joint6_deg.angle + 0.01)
		manipulator -> joint6_deg.angle += velocity_plan(&manipulator -> joint6_deg);
	else if(manipulator -> joint6_deg.angle_target + manipulator -> joint6_deg.angle_init < manipulator -> joint6_deg.angle - 0.01)
		manipulator -> joint6_deg.angle -= velocity_plan(&manipulator -> joint6_deg); 
	manipulator -> joint6_deg.rad = manipulator -> joint6_deg.angle * AtR;
}

void joint0_Ctrl_left(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch2 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint0_deg.angle_target -= -0.03;
//	else if(rc_Ctrl.rc.ch2 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint0_deg.angle_target += -0.03;

	if(manipulator -> joint0_deg.angle_target + manipulator -> joint0_deg.angle_init > manipulator -> joint0_deg.angle + 0.01)
		manipulator -> joint0_deg.angle += velocity_plan(&manipulator -> joint0_deg);
	else if(manipulator -> joint0_deg.angle_target + manipulator -> joint0_deg.angle_init < manipulator -> joint0_deg.angle - 0.01)
		manipulator -> joint0_deg.angle -= velocity_plan(&manipulator -> joint0_deg); 
	manipulator -> joint0_deg.rad = manipulator -> joint0_deg.angle * AtR;
}

void joint1_Ctrl_left(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch3 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint1_deg.angle_target += 0.01*sensitivity;
//	else if(rc_Ctrl.rc.ch3 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint1_deg.angle_target -= 0.01*sensitivity;

	if(manipulator -> joint1_deg.angle_target + manipulator -> joint1_deg.angle_init > manipulator -> joint1_deg.angle + 0.01)
		manipulator -> joint1_deg.angle += velocity_plan(&manipulator -> joint1_deg);
	else if(manipulator -> joint1_deg.angle_target + manipulator -> joint1_deg.angle_init < manipulator -> joint1_deg.angle - 0.01)
		manipulator -> joint1_deg.angle -= velocity_plan(&manipulator -> joint1_deg);
	manipulator -> joint1_deg.rad = manipulator -> joint1_deg.angle * AtR;
}

void joint2_Ctrl_left(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch0 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint2_deg.angle_target += 0.01;
//	else if(rc_Ctrl.rc.ch0 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint2_deg.angle_target -= 0.01;
	
	if(manipulator -> joint2_deg.angle_target + manipulator -> joint2_deg.angle_init > manipulator -> joint2_deg.angle + 0.01)
		manipulator -> joint2_deg.angle += velocity_plan(&manipulator -> joint2_deg);
	else if(manipulator -> joint2_deg.angle_target + manipulator -> joint2_deg.angle_init < manipulator -> joint2_deg.angle - 0.01)
		manipulator -> joint2_deg.angle -= velocity_plan(&manipulator -> joint2_deg); 
	manipulator -> joint2_deg.rad = manipulator -> joint2_deg.angle * AtR;
}

void joint3_Ctrl_left(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch1 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint3_deg.angle_target += -0.01;
//	else if(rc_Ctrl.rc.ch1 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint3_deg.angle_target -= -0.01;

	if(manipulator -> joint3_deg.angle_target + manipulator -> joint3_deg.angle_init > manipulator -> joint3_deg.angle + 0.01)
		manipulator -> joint3_deg.angle += velocity_plan(&manipulator -> joint3_deg);
	else if(manipulator -> joint3_deg.angle_target + manipulator -> joint3_deg.angle_init < manipulator -> joint3_deg.angle - 0.01)
		manipulator -> joint3_deg.angle -= velocity_plan(&manipulator -> joint3_deg); 
	manipulator -> joint3_deg.rad = manipulator -> joint3_deg.angle * AtR;
}

void joint4_Ctrl_left(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch2 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint4_deg.angle_target += 0.01;
//	else if(rc_Ctrl.rc.ch2 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint4_deg.angle_target -= 0.01;
	
	if(manipulator -> joint4_deg.angle_target + manipulator -> joint4_deg.angle_init > manipulator -> joint4_deg.angle + 0.01)
		manipulator -> joint4_deg.angle += velocity_plan(&manipulator -> joint4_deg);
	else if(manipulator -> joint4_deg.angle_target + manipulator -> joint4_deg.angle_init < manipulator -> joint4_deg.angle - 0.01)
		manipulator -> joint4_deg.angle -= velocity_plan(&manipulator -> joint4_deg); 
	manipulator -> joint4_deg.rad = manipulator -> joint4_deg.angle * AtR;
}

void joint5_Ctrl_left(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch3 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint5_deg.angle_target -= 0.01;
//	else if(rc_Ctrl.rc.ch3 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint5_deg.angle_target += 0.01;
	
	if(manipulator -> joint5_deg.angle_target + manipulator -> joint5_deg.angle_init > manipulator -> joint5_deg.angle + 0.01)
		manipulator -> joint5_deg.angle += velocity_plan(&manipulator -> joint5_deg);
	else if(manipulator -> joint5_deg.angle_target + manipulator -> joint5_deg.angle_init < manipulator -> joint5_deg.angle - 0.01)
		manipulator -> joint5_deg.angle -= velocity_plan(&manipulator -> joint5_deg); 
	manipulator -> joint5_deg.rad = manipulator -> joint5_deg.angle * AtR;
}

void joint6_Ctrl_left(Manipulator_t* manipulator){
//	if(rc_Ctrl.rc.ch0 > =  1124 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint6_deg.angle_target += 0.01;
//	else if(rc_Ctrl.rc.ch0 < =  924 && rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 3 && manipulator -> controller_mapping_flag % 2 == 0)
//		manipulator -> joint6_deg.angle_target -= 0.01;
	
	/* 自定义控制器模式 */
	if(Custom.image_recv.Coordinate.left_index_switch == 1 && manipulator -> controller_mapping_flag % 2 == 1)
		manipulator -> joint6_deg.angle_target -= 0.05 * manipulator -> joint6_deg.sensitivity;
	else if(Custom.image_recv.Coordinate.left_index_switch == -1 && manipulator -> controller_mapping_flag % 2 == 1)
		manipulator -> joint6_deg.angle_target += 0.05 * manipulator -> joint6_deg.sensitivity;
	
	if(manipulator -> joint6_deg.angle_target + manipulator -> joint6_deg.angle_init > manipulator -> joint6_deg.angle + 0.01)
		manipulator -> joint6_deg.angle += velocity_plan(&manipulator -> joint6_deg);
	else if(manipulator -> joint6_deg.angle_target + manipulator -> joint6_deg.angle_init < manipulator -> joint6_deg.angle - 0.01)
		manipulator -> joint6_deg.angle -= velocity_plan(&manipulator -> joint6_deg); 
	manipulator -> joint6_deg.rad = manipulator -> joint6_deg.angle * AtR;
}


/* 夹爪角度测试函数 */
void Clamp_jaw_angle_test(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left){
	manipulator_right -> clamp_jaw.STS_3215[0].position = test_angle0;
	manipulator_right -> clamp_jaw.STS_3215[1].position = test_angle1;

	manipulator_left -> clamp_jaw.STS_3215[0].position = test_angle2;
	manipulator_left -> clamp_jaw.STS_3215[1].position = test_angle3;

	// manipulator_right -> clamp_jaw.STS_3215[0].position = 1500 + open_angle_right;
	// manipulator_right -> clamp_jaw.STS_3215[1].position = 1500 - open_angle_right;
	
	// manipulator_left -> clamp_jaw.STS_3215[0].position = 1450 - open_angle_left;
	// manipulator_left -> clamp_jaw.STS_3215[1].position = 1525 + open_angle_left;
	
	if(tim14.ClockTime % 120 == 0){
		Feetech_STS_control(&manipulator_right -> clamp_jaw.STS_3215[0]);
	}
	if(tim14.ClockTime % 120 == 30){
		Feetech_STS_control(&manipulator_right -> clamp_jaw.STS_3215[1]);
	}
	
	if(tim14.ClockTime % 120 == 60){
		Feetech_STS_control(&manipulator_left -> clamp_jaw.STS_3215[0]);
	}
	if(tim14.ClockTime % 120 == 90){
		Feetech_STS_control(&manipulator_left -> clamp_jaw.STS_3215[1]);
	}
}

/**
* @brief 关节执行函数
  */ 
void joint_Ctrl(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left){
	/* 电机角度解算 */
	if(Custom.image_recv.Coordinate.isonline == 1){
		joints_data_filter(manipulator_right, manipulator_left, &Custom);
		
		/* 右臂自定义控制器映射 */
		if(manipulator_right -> controller_mapping_flag % 2 == 1){
			manipulator_right -> joint0_deg.angle = manipulator_right -> joint0_deg.zero_point * RtA + (-1.5) * (manipulator_right -> joint0_deg.cc_recv - manipulator_right -> joint0_deg.cc_init);
			manipulator_right -> joint1_deg.rad = (manipulator_right -> joint1_deg.zero_point + (2.1) * (manipulator_right -> joint1_deg.cc_recv - manipulator_right -> joint1_deg.cc_init) * AtR);
			manipulator_right -> joint2_deg.rad = (manipulator_right -> joint2_deg.zero_point + (-1.75) * (manipulator_right -> joint2_deg.cc_recv - manipulator_right -> joint2_deg.cc_init) * AtR);
			manipulator_right -> joint3_deg.rad = (manipulator_right -> joint3_deg.zero_point + (-1.4) * (manipulator_right -> joint3_deg.cc_recv - manipulator_right -> joint3_deg.cc_init) * AtR);
			manipulator_right -> joint4_deg.rad = (manipulator_right -> joint4_deg.zero_point + (-1) * (manipulator_right -> joint4_deg.cc_recv - manipulator_right -> joint4_deg.cc_init) * AtR);
			manipulator_right -> joint5_deg.rad = (manipulator_right -> joint5_deg.zero_point + (2.5) * (manipulator_right -> joint5_deg.cc_recv - manipulator_right -> joint5_deg.cc_init) * AtR);			
			/* joint6数据为1是正转，数据为-1时逆转，数据为0时不动 */
			if(Custom.image_recv.Coordinate.right_middle_switch == 0){
				joint6_Ctrl_right(manipulator_right);
			}	
		}
		else if(manipulator_right -> controller_mapping_flag % 2 == 0){
			joint0_Ctrl_right(manipulator_right);
			joint1_Ctrl_right(manipulator_right);
			joint2_Ctrl_right(manipulator_right);
			joint3_Ctrl_right(manipulator_right);
			joint4_Ctrl_right(manipulator_right);
			joint5_Ctrl_right(manipulator_right);
			joint6_Ctrl_right(manipulator_right);
		}
		
		/* 左臂自定义控制器映射 */
		if(manipulator_left -> controller_mapping_flag % 2 == 1){
			manipulator_left -> joint0_deg.angle = manipulator_left -> joint0_deg.zero_point * RtA + (-2.0) * (manipulator_left -> joint0_deg.cc_recv - manipulator_left -> joint0_deg.cc_init);
			manipulator_left -> joint1_deg.rad = (manipulator_left -> joint1_deg.zero_point + (2.1) * (manipulator_left -> joint1_deg.cc_recv - manipulator_left -> joint1_deg.cc_init) * AtR);
			manipulator_left -> joint2_deg.rad = (manipulator_left -> joint2_deg.zero_point + (-1.75) * (manipulator_left -> joint2_deg.cc_recv - manipulator_left -> joint2_deg.cc_init) * AtR);
			manipulator_left -> joint3_deg.rad = (manipulator_left -> joint3_deg.zero_point + (1.4) * (manipulator_left -> joint3_deg.cc_recv - manipulator_left -> joint3_deg.cc_init) * AtR);
			manipulator_left -> joint4_deg.rad = (manipulator_left -> joint4_deg.zero_point + (-1) * (manipulator_left -> joint4_deg.cc_recv - manipulator_left -> joint4_deg.cc_init) * AtR);
			manipulator_left -> joint5_deg.rad = (manipulator_left -> joint5_deg.zero_point + (2.5) * (manipulator_left -> joint5_deg.cc_recv - manipulator_left -> joint5_deg.cc_init) * AtR);
			/* joint6数据为1是正转，数据为-1时逆转，数据为0时不动 */
			if(Custom.image_recv.Coordinate.left_middle_switch == 0){
				joint6_Ctrl_left(manipulator_left);
			}
		}
		else if(manipulator_left -> controller_mapping_flag % 2 == 0){
			joint0_Ctrl_left(manipulator_left);
			joint1_Ctrl_left(manipulator_left);
			joint2_Ctrl_left(manipulator_left);
			joint3_Ctrl_left(manipulator_left);
			joint4_Ctrl_left(manipulator_left);
			joint5_Ctrl_left(manipulator_left);
			joint6_Ctrl_left(manipulator_left);
		}
	}
	else if(rc_Ctrl.isOnline == 1){
		joint0_Ctrl_right(manipulator_right);
		joint1_Ctrl_right(manipulator_right);
		joint2_Ctrl_right(manipulator_right);
		joint3_Ctrl_right(manipulator_right);
		joint4_Ctrl_right(manipulator_right);
		joint5_Ctrl_right(manipulator_right);
		joint6_Ctrl_right(manipulator_right);
		
		joint0_Ctrl_left(manipulator_left);
		joint1_Ctrl_left(manipulator_left);
		joint2_Ctrl_left(manipulator_left);
		joint3_Ctrl_left(manipulator_left);
		joint4_Ctrl_left(manipulator_left);
		joint5_Ctrl_left(manipulator_left);
		joint6_Ctrl_left(manipulator_left);
	}
	
//		all_joints_limit(manipulator_right);//右臂限幅
//		all_joints_limit(manipulator_left);//左臂限幅
	
//最终输出
	if(tim14.ClockTime > 4000){
		if(rc_Ctrl.isOnline == 1){
			if(tim14.ClockTime % 7 == 0){
				ctrl_motor1(&can1,&manipulator_right->Dm_4340_joint1,manipulator_right->joint1_deg.rad,0,175,2.0,1.0);
				ctrl_motor1(&can2,&manipulator_left->Dm_4340_joint1,manipulator_left->joint1_deg.rad,0,175,2.0,1.0);
			}
			if(tim14.ClockTime % 7 == 1){
				ctrl_motor1(&can1,&manipulator_right->Dm_4340_joint2,manipulator_right->joint2_deg.rad,0,85,0.6,0);
				ctrl_motor1(&can2,&manipulator_left->Dm_4340_joint2,manipulator_left->joint2_deg.rad,0,30,0.6,0);
			}
			if(tim14.ClockTime % 7 == 2){
				ctrl_motor1(&can1,&manipulator_right->Dm_8006_joint3,manipulator_right->joint3_deg.rad,0,85,1.6,0);
				ctrl_motor1(&can2,&manipulator_left->Dm_8006_joint3,manipulator_left->joint3_deg.rad,0,85,1.6,0);
			}
			if(tim14.ClockTime % 7 == 3){
				ctrl_motor1(&can1,&manipulator_right->Dm_4310_joint4,manipulator_right->joint4_deg.rad,0,30,0.6,0);
				ctrl_motor1(&can2,&manipulator_left->Dm_4310_joint4,manipulator_left->joint4_deg.rad,0,30,0.6,0);
			}
			if(tim14.ClockTime % 7 == 4){
				ctrl_motor1(&can1,&manipulator_right->Dm_4310_joint5,manipulator_right->joint5_deg.rad,0,30,0.6,0);
				ctrl_motor1(&can2,&manipulator_left->Dm_4310_joint5,manipulator_left->joint5_deg.rad,0,30,0.6,0);
			}
			if(tim14.ClockTime % 7 == 5){
				ctrl_motor1(&can1,&manipulator_right->Dm_4310_joint6,manipulator_right->joint6_deg.rad,0,30,0.6,0);
				ctrl_motor1(&can2,&manipulator_left->Dm_4310_joint6,manipulator_left->joint6_deg.rad,0,30,0.6,0);
			}
			if(tim14.ClockTime % 7 == 6){
				RPDO2_pos(&can1,500);
				RPDO1_pos(&can1,manipulator_right->joint0_deg.angle);		//位置模式(绝对位置+立即执行)+位置模式+目标位置
				RPDO2_pos(&can2,500);
				RPDO1_pos(&can2,manipulator_left->joint0_deg.angle);		//位置模式(绝对位置+立即执行)+位置模式+目标位置
			}
			clamp_jaw_data_send(&manipulator_right -> clamp_jaw, &manipulator_left -> clamp_jaw);
		}
		if(rc_Ctrl.isOnline == 0){
			if(tim14.ClockTime % 3 == 0){
				ctrl_motor1(&can1,&manipulator_right -> Dm_4340_joint1, 0, 0, 0, 0.3, 0);
				manipulator_right -> joint1_deg.rad = manipulator_right -> Dm_4340_joint1.pos;
				manipulator_right -> joint1_deg.angle = manipulator_right -> joint1_deg.rad * RtA;
				manipulator_right -> joint1_deg.angle_target = manipulator_right -> Dm_4340_joint1.pos * RtA - manipulator_right -> joint1_deg.angle_init;
				ctrl_motor1(&can2,&manipulator_left -> Dm_4340_joint1, 0, 0, 0, 0.3, 0);
				manipulator_left -> joint1_deg.rad = manipulator_left -> Dm_4340_joint1.pos;
				manipulator_left -> joint1_deg.angle = manipulator_left -> joint1_deg.rad * RtA;
				manipulator_left -> joint1_deg.angle_target = manipulator_left -> Dm_4340_joint1.pos * RtA - manipulator_left -> joint1_deg.angle_init;
				
				ctrl_motor1(&can1,&manipulator_right -> Dm_8006_joint3, 0, 0, 0, 1.5, 0);
				manipulator_right -> joint3_deg.rad = manipulator_right -> Dm_8006_joint3.pos;
				manipulator_right -> joint3_deg.angle = manipulator_right -> joint3_deg.rad * RtA;
				manipulator_right -> joint3_deg.angle_target = manipulator_right -> Dm_8006_joint3.pos * RtA - manipulator_right -> joint3_deg.angle_init;
				ctrl_motor1(&can2,&manipulator_left -> Dm_8006_joint3, 0, 0, 0, 1.5, 0);
				manipulator_left -> joint3_deg.rad = manipulator_left -> Dm_8006_joint3.pos;
				manipulator_left -> joint3_deg.angle = manipulator_left -> joint3_deg.rad * RtA;
				manipulator_left -> joint3_deg.angle_target = manipulator_left -> Dm_8006_joint3.pos * RtA - manipulator_left -> joint3_deg.angle_init;
			}
			if(tim14.ClockTime % 3 == 1){
				RPDO2_pos(&can1,0);
				manipulator_right -> joint0_deg.angle = MOTOR_ONE.pos;
				manipulator_right -> joint0_deg.angle_target = MOTOR_ONE.pos - manipulator_right -> joint0_deg.angle_init;
				RPDO2_pos(&can2,0);
				manipulator_left -> joint0_deg.angle = MOTOR_TWO.pos;
				manipulator_left -> joint0_deg.angle_target = MOTOR_TWO.pos - manipulator_left -> joint0_deg.angle_init;
				
				ctrl_motor1(&can1,&manipulator_right->Dm_4310_joint5,0,0,0,0.3,0);
				manipulator_right->joint5_deg.rad = manipulator_right -> Dm_4310_joint5.pos;
				manipulator_right -> joint5_deg.angle = manipulator_right -> joint5_deg.rad * RtA;
				manipulator_right -> joint5_deg.angle_target = manipulator_right -> Dm_4310_joint5.pos * RtA - manipulator_right -> joint5_deg.angle_init;
				ctrl_motor1(&can2,&manipulator_left->Dm_4310_joint5,0,0,0,0.3,0);
				manipulator_left->joint5_deg.rad = manipulator_left -> Dm_4310_joint5.pos;
				manipulator_left -> joint5_deg.angle = manipulator_left -> joint5_deg.rad * RtA;
				manipulator_left -> joint5_deg.angle_target = manipulator_left -> Dm_4310_joint5.pos * RtA - manipulator_left -> joint5_deg.angle_init;
				
				ctrl_motor1(&can1,&manipulator_right->Dm_4310_joint6,0,0,0,0.3,0);
				manipulator_right->joint6_deg.rad = manipulator_right -> Dm_4310_joint6.pos;
				manipulator_right -> joint6_deg.angle = manipulator_right -> joint6_deg.rad * RtA;
				manipulator_right -> joint6_deg.angle_target = manipulator_right -> Dm_4310_joint6.pos * RtA - manipulator_right -> joint6_deg.angle_init;
				ctrl_motor1(&can2,&manipulator_left->Dm_4310_joint6,0,0,0,0.3,0);
				manipulator_left->joint6_deg.rad = manipulator_left -> Dm_4310_joint6.pos;
				manipulator_left -> joint6_deg.angle = manipulator_left -> joint6_deg.rad * RtA;
				manipulator_left -> joint6_deg.angle_target = manipulator_left -> Dm_4310_joint6.pos * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			if(tim14.ClockTime % 3 == 2){
				ctrl_motor1(&can1,&manipulator_right->Dm_4340_joint2,0,0,0,0.3,0);
				manipulator_right->joint2_deg.rad = manipulator_right -> Dm_4340_joint2.pos;
				manipulator_right -> joint2_deg.angle = manipulator_right -> joint2_deg.rad * RtA;
				manipulator_right -> joint2_deg.angle_target = manipulator_right -> Dm_4340_joint2.pos * RtA - manipulator_right -> joint2_deg.angle_init;
				ctrl_motor1(&can2,&manipulator_left->Dm_4340_joint2,0,0,0,0.3,0);
				manipulator_left->joint2_deg.rad = manipulator_left -> Dm_4340_joint2.pos;
				manipulator_left -> joint2_deg.angle = manipulator_left -> joint2_deg.rad * RtA;
				manipulator_left -> joint2_deg.angle_target = manipulator_left -> Dm_4340_joint2.pos * RtA - manipulator_left -> joint2_deg.angle_init;
				
				ctrl_motor1(&can1,&manipulator_right -> Dm_4310_joint4,0,0,0,0.3,0);
				manipulator_right -> joint4_deg.rad = manipulator_right -> Dm_4310_joint4.pos;
				manipulator_right -> joint4_deg.angle = manipulator_right -> joint4_deg.rad * RtA;
				manipulator_right -> joint4_deg.angle_target = manipulator_right -> Dm_4310_joint4.pos * RtA - manipulator_right -> joint4_deg.angle_init;
				ctrl_motor1(&can2,&manipulator_left -> Dm_4310_joint4,0,0,0,0.3,0);
				manipulator_left -> joint4_deg.rad = manipulator_left -> Dm_4310_joint4.pos;
				manipulator_left -> joint4_deg.angle = manipulator_left -> joint4_deg.rad * RtA;
				manipulator_left -> joint4_deg.angle_target = manipulator_left -> Dm_4310_joint4.pos * RtA - manipulator_left -> joint4_deg.angle_init;
			}
		}
	}
}

void joints_data_filter(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, custom_robot_data_t* custom){
	manipulator_right -> joint0_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint0_right, &joint0_angle_right);
	manipulator_right -> joint1_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint1_right, &joint1_angle_right);
	manipulator_right -> joint2_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint2_right, &joint2_angle_right);
	manipulator_right -> joint3_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint3_right, &joint3_angle_right);
	manipulator_right -> joint4_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint4_right, &joint4_angle_right);
	manipulator_right -> joint5_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint5_right, &joint5_angle_right);
	manipulator_right -> joint6_deg.cc_recv = custom -> image_recv.Coordinate.joint6_right;

	manipulator_left -> joint0_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint0_left, &joint0_angle_left);
	manipulator_left -> joint1_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint1_left, &joint1_angle_left);
	manipulator_left -> joint2_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint2_left, &joint2_angle_left);
	manipulator_left -> joint3_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint3_left, &joint3_angle_left);
	manipulator_left -> joint4_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint4_left, &joint4_angle_left);
	manipulator_left -> joint5_deg.cc_recv = LPFilter(custom -> image_recv.Coordinate.joint5_left, &joint5_angle_left);
	manipulator_left -> joint6_deg.cc_recv = custom -> image_recv.Coordinate.joint6_left;
}