#ifndef INTERACTION_IMAGE_H__
#define INTERACTION_IMAGE_H__

#include "stm32h7xx.h"
#include "interaction.h"
#include "driver_usart.h"
typedef float float32_t;
#define controller_data_lenth  30 //自定义控制器数据长度
#define rc_data_lenth  17 //遥控数据长度
#define USART2_RXBUF_SIZE 7+controller_data_lenth+2+5+10//取最大
#define USART2_TXBUF_SIZE 7+controller_data_lenth+2+10//取最大

#define  right_thumb_switch  switches[0]	//右拇指开关
#define  right_index_switch  joint6_right		//右食指开关，使用Joint6数据
#define  right_middle_switch switches[1]	//右中指开关
#define  left_thumb_switch   switches[2] 		//左拇指开关
#define  left_index_switch   joint6_left		//左食指开关，使用Joint6数据
#define  left_middle_switch	 switches[3] 	//左中指开关

extern int16_t recv_test;
extern UART_RxBuffer uart2_buffer;
uint8_t imagetrans_callback(uint8_t * recBuffer, uint16_t len);

extern int Joint0,Joint1,Joint2,Joint3,Joint4,Joint5,Joint6;

typedef __packed struct //自定义发送结构体
{
	int sx;
	int sy;
	int sz;
	int16_t x_speed;
	int16_t y_speed;
	int16_t z_speed;
	int16_t yaw;
	int16_t pitch;
	int16_t roll;
}coordinates_send;

typedef __packed struct //自定义控制器发送结构体
{
	int joint1;
	int joint0;
	int joint2;
	int joint3;
	
	int roll1;
	int pitch3;
	int roll2;
}angledata_send;

typedef __packed struct //自定义接收结构体
{
	float joint0_right;
	float joint1_right;
	float joint2_right;
	float joint3_right;
	float joint4_right;
	float joint5_right;
	int8_t joint6_right;
	
	float joint0_left;
	float joint1_left;
	float joint2_left;
	float joint3_left;
	float joint4_left;
	float joint5_left;
	int8_t joint6_left;
	uint8_t switches[4];
	
	uint8_t  cc_circle;
	uint8_t isonline;
}coordinates_recv;


typedef struct
{
	__packed struct
	{
		//< frame_header(5-byte) 帧头部分
		frame_header_t image_frame_header;  
		//< cmd_id(2-byte) 命令码
		uint16_t cmd_id;
		//< 自定义数据(-byte) 
		uint8_t all_switch;//映射状态send_flag
		angledata_send angle_send;
		uint8_t key_send;
		uint8_t CRC16[2];
	
	}image_send;
	__packed struct{
		coordinates_recv Coordinate;//坐标
	}image_recv;

}custom_robot_data_t;

uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
extern unsigned char recv_Buffer[USART2_RXBUF_SIZE];

extern custom_robot_data_t Custom;//自定义控制器

extern int image_count;
#endif