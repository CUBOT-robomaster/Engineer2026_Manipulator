#ifndef __FILTER_H__
#define __FILTER_H__

#include "main.h"

struct LowPassFilter_Info{	
	float filter_coefficient;
	float last_output;
	float output;
	float sampling;	
};

float LPFilter(float sampling ,struct LowPassFilter_Info *LPF);

extern struct LowPassFilter_Info joint0_angle_right;
extern struct LowPassFilter_Info joint1_angle_right;
extern struct LowPassFilter_Info joint2_angle_right;
extern struct LowPassFilter_Info joint3_angle_right;
extern struct LowPassFilter_Info joint4_angle_right;
extern struct LowPassFilter_Info joint5_angle_right;
extern struct LowPassFilter_Info joint6_angle_right;

extern struct LowPassFilter_Info joint0_angle_left;
extern struct LowPassFilter_Info joint1_angle_left;
extern struct LowPassFilter_Info joint2_angle_left;
extern struct LowPassFilter_Info joint3_angle_left;
extern struct LowPassFilter_Info joint4_angle_left;
extern struct LowPassFilter_Info joint5_angle_left;
extern struct LowPassFilter_Info joint6_angle_left;
#endif
