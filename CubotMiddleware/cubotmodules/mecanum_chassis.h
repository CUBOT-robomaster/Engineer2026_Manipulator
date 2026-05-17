#ifndef MECANUMCHASSIS_H
#define MECANUMCHASSIS_H
#include "motor.h"
#include "pid.h"
#include "dr16.h"
#include "attitude.h"
#include "hardware_config.h"
#include <dr16.h>
#include <vt13.h>
#include "auto.h"
#include "joint_control.h"

#define USART4_RXBUF_SIZE 30
#define USART4_TXBUF_SIZE 30

extern unsigned char Usart4_RxBuffer[USART4_RXBUF_SIZE];
extern unsigned char Usart4_TxBuffer[USART4_TXBUF_SIZE];
extern UART_RxBuffer uart4_buffer;

typedef struct MecanumChassis_T			
{
struct 
	{
		Motor motor[4];
		BasePID_Object RunPID[4];
	}Motors;

	struct 
	{
		int16_t Vx;			
		int16_t Vy;		    
		int16_t Vomega;		
		float	Vx_Sensitivity;
		float	Vy_Sensitivity;
		float	Vomega_Sensitivity;		
	}Movement;
}MecanumChassis_t;

typedef struct{
	uint8_t header_0;
	uint8_t header_1;
	int32_t X_Integ;					//float
	int32_t Y_Integ;					//float
	int16_t vt13_mouse_x;			//鼠标位移
	uint16_t rc_sw;
	uint8_t rc_s1;
	uint8_t V_flag;
	uint8_t rc_isOnline;
	uint8_t control_flags[3];	//八个标志位
	uint8_t land_flag;				//抬升标志位
	uint16_t data_check_num;	//检测数据是否更新，数值为计时器的0.1倍
	
}MecanumChassis_Send;//需要发送给底盘的数据

typedef struct{
	uint8_t lift_complish_flag;	//底盘抬升成功的标志位，确保电机正常复位
	int8_t back_flag;
}MecanumChassis_Recv;//底盘接收的数据

extern float rotated_vy;
extern float rotated_vx;
extern uint8_t mecanum_recv_flag;
extern uint16_t mecanum_recv_count;
extern MecanumChassis_Recv mecanum_Recv;
void Mecanum_data_Send(UART_HandleTypeDef* huart_x,unsigned char* pBuffer);
void Mecanum_data_Recv(uint8_t * recBuffer, uint16_t len);
uint8_t mecanum_callback(uint8_t * recBuffer, uint16_t len);
#endif
