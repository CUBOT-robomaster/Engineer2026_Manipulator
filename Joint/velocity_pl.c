#include "joint_control.h"
#include "mit.h"
#include "dr16.h"
#include "motor.h"
#include "driver_timer.h"
#include "AIM_PDO.h"
#include "interaction_image.h"
#include "controllerl.h"

float speed;

/**
  * @brief 关节角速度规划
  */
  
float f_abs(float angle){
	if(angle >= 0)
		return angle;
	else 
		return -1 * angle;
}
float velocity_plan(joint_angle_t* joint){
	if(fabs((joint -> angle_target + joint -> angle_init) - joint -> angle) >= 10)
		joint->velocity +=   joint -> Accel;

	else if(fabs((joint -> angle_target + joint -> angle_init) - joint -> angle) < 10)
		joint->velocity -=  joint -> Accel;
	
	if(joint -> velocity > joint -> max_velocity)
		joint -> velocity = joint -> max_velocity;
	if(joint -> velocity < joint -> min_velocity)
		joint -> velocity = joint -> min_velocity;
	return joint -> velocity;
}	 











