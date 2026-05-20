#ifndef __AUTO_H_
#define __AUTO_H_
#include "tim.h"
#include "stdint.h"
#include "joint_control.h"
#include "control_logic.h"
#include "vt13.h"
#include "servo.h"
#include "interaction_image.h"
#include "controllerl.h"
#include "mit.h"
#define servo_back_time_limit 200				//舵机转头最大时间
#define Mapping_Filter_Limit_Time 150			//映射模式按鍵消抖时间
#define Mapping_Exit_Filter_Limit_Time 20		//退出映射模式按键消抖时间
#define Clamp_Jaw_Close_Filter_Limit_Time 20	//夹爪闭合按键消抖时间
#define servo_pitch_sensitivity 5.0f			//舵机pitch轴灵敏度


typedef struct{
	uint8_t		land_flag;						//登岛初始化标志位
	uint32_t 	land_count;						//登岛复位计时

	uint8_t 	pre_lift_flag;					//发送给底盘的标志位，让底盘抬升，2下降，1抬升
	uint8_t 	lift_complish_flag;				//底盘抬升成功的标志位，确保电机正常复位

	int8_t 		step_down_flag;					//台阶下降标志位
	int8_t 		servo_back_flag;				//图传转头标志位
	uint16_t 	servo_back_count;				//图传转头计时

	int8_t 		scope_mode_flag;				//倍镜模式标志位
	uint32_t 	scope_mode_count;				//倍镜模式计时
	uint32_t 	scope_filter_count;				//倍镜模式按键消抖计时

	uint8_t 	pre_mapping_flag;				//准备进行映射标志位
	uint32_t 	pre_mapping_count;
	uint8_t 	mapping_exit_flag;				//退出映射模式标志位
	uint16_t 	mapping_exit_count;				//退出映射模式计时

	uint8_t 	zero_reset_flag;				//复位至零点标志位

	uint8_t 	right_clamp_jaw_key_flag;		//右夹爪按键标志位
	uint16_t	right_clamp_jaw_key_count;		//右夹爪按键计时
	uint8_t 	left_clamp_jaw_key_flag;		//左夹爪按键标志位
	uint16_t	left_clamp_jaw_key_count;		//左夹爪按键计时

	uint8_t 	motor_start_mode_flag;			//电机使能标志位
	uint16_t 	motor_start_mode_count;			//电机使能模式计时

	uint8_t		lifting_auto_flag;				//抬升自动标志位，此时无法手动控制抬升
}auto_control_flags;

/* 测试用变量 */
typedef struct{
	uint8_t right_thumb_flag;
	int8_t right_index_flag;
	uint8_t right_middle_flag;
	uint8_t left_thumb_flag;
	int8_t left_index_flag;
	uint8_t left_middle_flag;
}finger_gesture_flags;

void zero_point_reset(Manipulator_t* manipulator);
void land_point_reset(Manipulator_t* manipulator);
void step_out_point_reset(Manipulator_t* manipulator);
void finger_data_test(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, custom_robot_data_t* custom, finger_gesture_flags* finger_flags);
void Auto_Control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags, Hiwonder_Servo* hiwo_data, custom_robot_data_t* custom);
void land_reset_control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags);
void servo_back_control(Manipulator_t *manipulator_right, Manipulator_t *manipulator_left, Hiwonder_Servo* hiwo_data, auto_control_flags* auto_flags);
void point_of_view_control(Hiwonder_Servo* hiwo_data, auto_control_flags* auto_flags);
void scope_mode_control(Hiwonder_Servo* hiwo_data, auto_control_flags* auto_flags);
void clamp_jaw_control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags, custom_robot_data_t* custom);
void lifting_control(auto_control_flags* auto_flags);
void Controller_mode_start(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags, custom_robot_data_t* custom);
void Controller_mode_exit(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags);
void motor_start_control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags);
extern auto_control_flags Auto_flags;
#endif