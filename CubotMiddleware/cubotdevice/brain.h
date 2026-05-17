#ifndef _BRAIN_H
#define _BRAIN_H
//#include <stm32h7xx_hal.h>
//#include "driver_usart.h"
//#include "hardware_config.h"
///**
//  * @brief  上位机大脑内核结构体
//  */
//typedef struct
//{
//	uint8_t CoreID;					//< 内核固定ID
//	uint8_t BrainMode;  		//< 请求机器人大脑内核切换的工作模式。
//	uint8_t BrainVelocity;  //< 请求机器人大脑内核切换的子弹初速度。
//	
//	struct 
//	{
//		float YawDeflectionAngle;
//		float PitchDeflectionAngle;
//		float Distance;
//		uint8_t IsFire;      //开火建议
//	}CoreInstruction; 

//	struct 
//	{
//		uint8_t Working;     //机器人大脑内核的相机工作状态 
//		uint8_t Connect;     //机器人大脑内核的相机连接状态
//		uint8_t Open;        //机器人大脑内核的打开状态
// 		uint8_t Init;        //机器人大脑内核的初始化状态
//	}CoreFlag;  //0关 1开	
//	
//	struct
//	{
//	  float sideway_msg;
//		float lifting_msg;
//		float protract_msg;
//	  float Yaw_msg;
//		float Pitch_msg;
//		float Roll_msg;
//	}Trajectory_msg;

//}BrainCore_t;         


///**
//  * @brief  上位机大脑结构体
//  */
//typedef struct
//{ 
//	uint8_t FrameType;
//	uint8_t FrameCoreID;
//	uint16_t Rx_count;
//  uint8_t auto_ore_flag;

//	uint8_t auto_ore_flag_ok;
//	BrainCore_t BrainCore[1024];
//	uint8_t recdata[1024];
//	uint16_t error;
//	uint8_t sum_msg;
//	union
//	{
//	 float data;
//	 uint8_t data_b[4];
//	}data_u;
//}CubotBrain_t;           


//uint8_t Brain_callback(uint8_t * recBuffer, uint16_t len);
//void Brain_DataUnpack(CubotBrain_t* Brain, uint8_t* recBuffer,uint16_t len);
//void auto_ore_assignment(Servo_t *servo,CubotBrain_t* brain);
//void Request_identification();
//extern CubotBrain_t Brain;
//extern uint8_t ore_begin_flag;
//extern uint32_t auto_ore_delay;
//extern uint32_t auto_ore_time;
#endif
