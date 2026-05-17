#include "servo.h"
#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "modbus.h"
#include "driver_timer.h"
#include "joint_control.h"

#define SERVO_Position_ADDRESS    128    	//位置控制地址（查阅飞特SM45BL舵机内存表）
#define SERVO_Velocity_ADDRESS    131    	//速度控制地址（查阅飞特SM45BL舵机内存表）
#define SERVO_Accelertion_ADDRESS  130   	//加速度控制地址（查阅飞特SM45BL舵机内存表）

Hiwonder_Servo hiwonder_Servo = {
	.yaw_servo.servo_id = 0,
	.yaw_servo.position = SERVO_FRONT_POSITION,
	.yaw_servo.move_time = 120,
	
	.pitch_servo.servo_id = 1,
	.pitch_servo.position = SERVO_UP_POSITION,
	.pitch_servo.move_time = 60,
	
	.roll_servo.servo_id = 2,
	.roll_servo.position = SERVO_ROLL_DOWN_POSITION,
	.roll_servo.move_time = 150,
};

uint8_t buf[13];
uint8_t buf_count = 0;

/* 幻尔总线舵机 */
uint8_t Hiwonder_buffer[10];
int16_t Hiwonder_control=500;
int16_t Hiwonder_move=0;

/* 飞特STS舵机 */
void Feetech_STS_control(Feetch_Data* FT_data){
	uint16_t check = 0;
	buf_count = 0;
	buf[buf_count++] = 0xFF;
	buf[buf_count++] = 0xFF;																	//两个0xFF为包头，表示有数据包
	buf[buf_count++] = FT_data -> servo_id;														//id
	buf[buf_count++] = 0x09;																	//指令包数据长度
	buf[buf_count++] = 0x03;																	//指令类型
	buf[buf_count++] = 0x2A;																	//写入首地址
	
	buf[buf_count++] = (uint8_t)(FT_data -> position);				//位置低字节
	buf[buf_count++] = (uint8_t)(FT_data -> position >> 8);			//位置高字节
	
	buf[buf_count++] = 0x00;																	//时间低字节
	buf[buf_count++] = 0x00;																	//时间高字节（当前控制模式下无实际意义，给0即可）
	
	buf[buf_count++] = (uint8_t)(FT_data -> velocity);				//速度低字节
	buf[buf_count++] = (uint8_t)(FT_data -> velocity >> 8);			//速度高字节
	
	for(int i = 2; i < 12; i++){
		check += buf[i];
	}
	if(check > 0xFF){
		check &= 0xFF;
	}
	check = ~(check);																					//校验和计算
	
	buf[buf_count++] = (uint8_t)check;												//校验和
	HAL_UART_Transmit_DMA(&huart5, buf, 13);									//发送指令
}

void clamp_jaw_data_send(Feetech_servo* FT_servo_right, Feetech_servo* FT_servo_left){

	if(tim14.ClockTime % 120 == 0){ 
		Feetech_STS_control(&FT_servo_right -> STS_3215[0]);
	}
	if(tim14.ClockTime % 120 == 30){
		Feetech_STS_control(&FT_servo_right -> STS_3215[1]);
	}
	if(tim14.ClockTime % 120 == 60){
		Feetech_STS_control(&FT_servo_left -> STS_3215[0]);
	}
	if(tim14.ClockTime % 120 == 90){
		Feetech_STS_control(&FT_servo_left -> STS_3215[1]);
	}
}

/* 幻尔舵机 */
uint8_t  servo_check_number(uint8_t buf[])
{   uint8_t i;
    uint16_t temp=0;
    for(i=2;i<buf[3]+2;i++){
    temp+=buf[i];
    
    }
    temp=~temp;
    i=(uint8_t)temp;
return i;
}
/* 位置模式 */
void servo_move(uint16_t id,uint16_t time,int16_t angle)
{   
    Hiwonder_buffer[0]=Hiwonder_buffer[1]=0x55;
    Hiwonder_buffer[2]=id;
    Hiwonder_buffer[3]=SERVO_MOVE_TIME_WRITE_Lenth;
    Hiwonder_buffer[4]=SERVO_MOVE_TIME_WRITE_Commandvalue;
    Hiwonder_buffer[5]=((int8_t )(angle));
    Hiwonder_buffer[6]=((int8_t )((angle)>>8));
    Hiwonder_buffer[7]=((int8_t )(time));
    Hiwonder_buffer[8]=((int8_t )(time>>8));
    Hiwonder_buffer[9]=servo_check_number(Hiwonder_buffer);
    HAL_UART_Transmit_DMA(&huart7,(unsigned char*)Hiwonder_buffer,10);
}

int16_t servo_angle_limit(int16_t angle, int16_t limit_min, int16_t limit_max){
	if(angle < limit_min){
		angle = limit_min;
	}
	else if(angle > limit_max){
		angle = limit_max;
	}
	return angle;
}

void hiwonder_servo_control(int32_t clock, Hiwonder_Servo* hiwonder_servo_data){
	if(clock % 120 == 0){
		/* yaw轴舵机 */
		servo_move(hiwonder_servo_data -> yaw_servo.servo_id, hiwonder_servo_data -> yaw_servo.move_time, hiwonder_servo_data -> yaw_servo.position);					//950正前方，175正后方
	}
	if(clock % 120 == 40){
		/* pitch轴舵机 */
		servo_move(hiwonder_servo_data -> pitch_servo.servo_id, hiwonder_servo_data -> pitch_servo.move_time, hiwonder_servo_data -> pitch_servo.position);		//475平视，275最低
	}
	if(clock % 120 == 80){
		/* roll轴舵机 */
		servo_move(hiwonder_servo_data -> roll_servo.servo_id, hiwonder_servo_data -> roll_servo.move_time, hiwonder_servo_data -> roll_servo.position);
	}
}
/* ID254为广播所有舵机 */