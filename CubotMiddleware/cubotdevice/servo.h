#ifndef __SERVO_H__
#define __SERVO_H__

#define SERVO_MOVE_TIME_WRITE_Commandvalue 1
#define SERVO_MOVE_TIME_WRITE_Lenth 7
#define SERVO_OR_MOTOR_MODE_WRITE_Commandvalue 29
#define SERVO_OR_MOTOR_MODE_WRITE_Lenth 7

#define SERVO_FRONT_POSITION 944
#define SERVO_BACK_POSITION 175
#define SERVO_UP_POSITION 510
#define SERVO_MIDDLE_POSITION 385
#define SERVO_DOWN_POSITION 275
#define SERVO_ROLL_UP_POSITION 205
#define SERVO_ROLL_DOWN_POSITION 570

#define SERVO_LEFT_ZERO_OPEN_ID_ZERO 1425
#define SERVO_LEFT_ZERO_CLOSE_ID_ZERO 1725
#define SERVO_RIGHT_ZERO_OPEN_ID_ONE 1550
#define SERVO_RIGHT_ZERO_CLOSE_ID_ONE 1275
#define SERVO_LEFT_ONE_OPEN_ID_TWO 1725
#define SERVO_LEFT_ONE_CLOSE_ID_TWO 1425
#define SERVO_RIGHT_ONE_OPEN_ID_THREE 1450
#define SERVO_RIGHT_ONE_CLOSE_ID_THREE 1725

#include "stdint.h"
#include "dr16.h"


/* 飞特舵机结构体 */
typedef struct{
	uint8_t servo_id;
	uint16_t position;
	uint16_t velocity;
}Feetch_Data;

typedef struct{
	Feetch_Data STS_3215[2];//飞特STS3215舵机
	uint8_t clamp_jaw_close_flag;		//夹爪闭合标志位
	uint16_t clamp_jaw_close_count;		//夹爪闭合计时
}Feetech_servo;

/* 幻尔舵机结构体 */
typedef struct{
	uint8_t servo_id;
	int16_t position;		//目标位置
	uint16_t move_time;		//移动到指定位置的时间
	float target_angle;		//目标角度（单位：度）
	float angle_send;		//发送给舵机的角度（单位：度）
}Hiwonder_Data;

typedef struct{
	Hiwonder_Data yaw_servo;
	Hiwonder_Data pitch_servo;
	Hiwonder_Data roll_servo;
}Hiwonder_Servo;

extern Hiwonder_Servo hiwonder_Servo;

void Feetech_STS_control(Feetch_Data* FT_data);
void servo_move(uint16_t id,uint16_t time,int16_t angle);
int16_t servo_angle_limit(int16_t angle, int16_t limit_min, int16_t limit_max);
void clamp_jaw_data_send(Feetech_servo* FT_servo_right, Feetech_servo* FT_servo_left);
void hiwonder_servo_control(int32_t clock, Hiwonder_Servo* hiwonder_servo_data);
#endif
