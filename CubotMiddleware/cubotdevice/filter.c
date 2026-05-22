#include "filter.h"

/* ÓÒ±ÛÂË²¨½á¹¹Ìå */
struct LowPassFilter_Info joint0_angle_right={
	.filter_coefficient=0.0025f,
	.last_output=0,
};

struct LowPassFilter_Info joint1_angle_right={
	.filter_coefficient=0.005f,
	.last_output=0,
};

struct LowPassFilter_Info joint2_angle_right={
	.filter_coefficient=0.01f,
	.last_output=0,
};

struct LowPassFilter_Info joint3_angle_right={
	.filter_coefficient=0.015f,
	.last_output=0,
};

struct LowPassFilter_Info joint4_angle_right={
	.filter_coefficient=0.0025f,
	.last_output=0,
};

struct LowPassFilter_Info joint5_angle_right={
	.filter_coefficient=0.0025f,
	.last_output=0,
};

struct LowPassFilter_Info joint6_angle_right={
 	.filter_coefficient=0.01f,
	.last_output=0,
};



/* ×ó±ÛÂË²¨½á¹¹Ìå */
struct LowPassFilter_Info joint0_angle_left={
	.filter_coefficient=0.0025f,
	.last_output=0,
};

struct LowPassFilter_Info joint1_angle_left={
	.filter_coefficient=0.005f,
	.last_output=0,
};

struct LowPassFilter_Info joint2_angle_left={
	.filter_coefficient=0.01f,
	.last_output=0,
};

struct LowPassFilter_Info joint3_angle_left={
	.filter_coefficient=0.015f,
	.last_output=0,
};

struct LowPassFilter_Info joint4_angle_left={
	.filter_coefficient=0.0025f,
	.last_output=0,
};

struct LowPassFilter_Info joint5_angle_left={
	.filter_coefficient=0.0025f,
	.last_output=0,
};

struct LowPassFilter_Info joint6_angle_left={
	.filter_coefficient=0.01f,
	.last_output=0,
};

float LPFilter(float sampling, struct LowPassFilter_Info *LPF){
	/* Ò»½×µÍÍ¨ÂË²¨Æ÷£ºp(n) = c¡¤q(n) + (1 - c)¡¤p(n - 1) */
	LPF -> sampling = sampling;
	LPF -> output = LPF -> filter_coefficient * LPF -> sampling + (1 - LPF -> filter_coefficient) * LPF -> last_output; 
	LPF -> last_output = LPF -> output;
	
	return LPF -> output ;
};