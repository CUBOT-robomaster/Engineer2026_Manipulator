#include "controllerl.h"
#include "joint_control.h"
#include "mit.h"
#include "dr16.h"
#include "motor.h"
#include "driver_timer.h"
#include "AIM_PDO.h"
#include "servo.h"
#include "vt13.h"
#include "filter.h"
controller_data Controller;
int CC_start_timer=0;
int CC_back_timer=0;
int8_t CC_back_flag=0;
/**
  * @brief 自定义控制器初始化函数
  */

//图传链路
void Customer_init_image_right(Manipulator_t* manipulator_right, custom_robot_data_t* Custom){
	/* 右臂自定义控制器初始化 */
	manipulator_right -> joint0_deg.cc_recv = Custom->image_recv.Coordinate.joint0_right;
	manipulator_right -> joint0_deg.cc_init = manipulator_right -> joint0_deg.cc_recv;
	joint0_angle_right.last_output = Custom -> image_recv.Coordinate.joint0_right;
	
	manipulator_right -> joint1_deg.cc_recv = Custom->image_recv.Coordinate.joint1_right;
	manipulator_right -> joint1_deg.cc_init = manipulator_right -> joint1_deg.cc_recv;
	joint1_angle_right.last_output = Custom -> image_recv.Coordinate.joint1_right;
	
	manipulator_right -> joint2_deg.cc_recv = Custom->image_recv.Coordinate.joint2_right;
	manipulator_right -> joint2_deg.cc_init = manipulator_right -> joint2_deg.cc_recv;
	joint2_angle_right.last_output = Custom -> image_recv.Coordinate.joint2_right;
	
	manipulator_right -> joint3_deg.cc_recv = Custom->image_recv.Coordinate.joint3_right;
	manipulator_right -> joint3_deg.cc_init = manipulator_right -> joint3_deg.cc_recv;
	joint3_angle_right.last_output = Custom -> image_recv.Coordinate.joint3_right;
	
	manipulator_right -> joint4_deg.cc_recv = Custom->image_recv.Coordinate.joint4_right;
	manipulator_right -> joint4_deg.cc_init = manipulator_right -> joint4_deg.cc_recv;
	joint4_angle_right.last_output = Custom -> image_recv.Coordinate.joint4_right;
	
	manipulator_right -> joint5_deg.cc_recv = Custom->image_recv.Coordinate.joint5_right;
	manipulator_right -> joint5_deg.cc_init = manipulator_right -> joint5_deg.cc_recv;
	joint5_angle_right.last_output = Custom -> image_recv.Coordinate.joint5_right;
	
	manipulator_right -> joint6_deg.cc_recv = Custom->image_recv.Coordinate.joint6_right;
	manipulator_right -> joint6_deg.cc_init = manipulator_right -> joint6_deg.cc_recv;
	joint6_angle_right.last_output = Custom -> image_recv.Coordinate.joint6_right;
}

void Customer_init_image_left(Manipulator_t* manipulator_left, custom_robot_data_t* Custom){
	/* 左臂自定义控制器初始化 */
	manipulator_left -> joint0_deg.cc_recv = Custom->image_recv.Coordinate.joint0_left;
	manipulator_left -> joint0_deg.cc_init = manipulator_left -> joint0_deg.cc_recv;
	joint0_angle_left.last_output = Custom -> image_recv.Coordinate.joint0_left;
	
	manipulator_left -> joint1_deg.cc_recv = Custom->image_recv.Coordinate.joint1_left;
	manipulator_left -> joint1_deg.cc_init = manipulator_left -> joint1_deg.cc_recv;
	joint1_angle_left.last_output = Custom -> image_recv.Coordinate.joint1_left;
	
	manipulator_left -> joint2_deg.cc_recv = Custom->image_recv.Coordinate.joint2_left;
	manipulator_left -> joint2_deg.cc_init = manipulator_left -> joint2_deg.cc_recv;
	joint2_angle_left.last_output = Custom -> image_recv.Coordinate.joint2_left;
	
	manipulator_left -> joint3_deg.cc_recv = Custom->image_recv.Coordinate.joint3_left;
	manipulator_left -> joint3_deg.cc_init = manipulator_left -> joint3_deg.cc_recv;
	joint3_angle_left.last_output = Custom -> image_recv.Coordinate.joint3_left;
	
	manipulator_left -> joint4_deg.cc_recv = Custom->image_recv.Coordinate.joint4_left;
	manipulator_left -> joint4_deg.cc_init = manipulator_left -> joint4_deg.cc_recv;
	joint4_angle_left.last_output = Custom -> image_recv.Coordinate.joint4_left;
	
	manipulator_left -> joint5_deg.cc_recv = Custom->image_recv.Coordinate.joint5_left;
	manipulator_left -> joint5_deg.cc_init = manipulator_left -> joint5_deg.cc_recv;
	joint5_angle_left.last_output = Custom -> image_recv.Coordinate.joint5_left;
	
	manipulator_left -> joint6_deg.cc_recv = Custom->image_recv.Coordinate.joint6_left;
	manipulator_left -> joint6_deg.cc_init = manipulator_left -> joint6_deg.cc_recv;
	joint6_angle_left.last_output = Custom -> image_recv.Coordinate.joint6_left;
}