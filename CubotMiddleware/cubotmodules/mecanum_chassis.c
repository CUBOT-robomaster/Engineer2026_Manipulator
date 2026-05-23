#include "mecanum_chassis.h"
#include <math.h>
#include "auto.h"
#include "driver_timer.h"
#include <string.h>
#define AtR 0.0174532f	
#define Mecanum_Datas_Send_Length 30
#define recv_count_max 3			
float rotated_vy;
float rotated_vx;

uint8_t mecanum_recv_flag = 1;		//接收标志位，为1时表示可以发送数据
uint16_t mecanum_recv_count = 0;	//接收计数

MecanumChassis_Send mecanum_Send = {
	.header_0 = 0xEF,
	.header_1 = 0xA1,
	.control_flags = 0,
};

MecanumChassis_Recv mecanum_Recv;

unsigned char Usart4_RxBuffer[USART4_RXBUF_SIZE];
unsigned char Usart4_TxBuffer[USART4_TXBUF_SIZE];

UART_RxBuffer uart4_buffer={
	.Data = Usart4_RxBuffer,
	.Size = USART4_RXBUF_SIZE
};

void Mecanum_data_Send(UART_HandleTypeDef* huart_x, unsigned char* pBuffer){
	mecanum_Send.X_Integ = (int32_t)(rc_Ctrl.Chassis_X_Integ * 1000 + vT13.Chassis_X_Integ * 1000);
	mecanum_Send.Y_Integ = (int32_t)(rc_Ctrl.Chassis_Y_Integ * 1000 + vT13.Chassis_Y_Integ * 1000);
	mecanum_Send.vt13_mouse_x = (vT13.mouse.x) * 3 + (rc_Ctrl.rc.ch2 - 1024) * 0.3;	//图传ch2是左拨杆上下
	mecanum_Send.rc_sw = rc_Ctrl.rc.sw;
	mecanum_Send.rc_s1 = rc_Ctrl.rc.s1;
	mecanum_Send.V_flag = rc_Ctrl.rc.s2;
	mecanum_Send.rc_isOnline = rc_Ctrl.isOnline;
	
	mecanum_Send.control_flags[0] = (vT13.key_ctrl_flag << 5) | (vT13.key_shift_flag << 4) | (vT13.key_R_flag << 3) | (vT13.key_Z_flag << 2) | (vT13.key_X_flag << 1) | (vT13.key_C_flag);
	mecanum_Send.control_flags[1] = (vT13.key_V_flag << 4) | (vT13.key_F_flag << 3) | (vT13.key_G_flag << 2) | ((Manipulator_Left.controller_mapping_flag % 2) << 1) | (Manipulator_Right.controller_mapping_flag % 2);
	mecanum_Send.control_flags[2] = 0;
	mecanum_Send.land_flag = Auto_flags.pre_lift_flag;
	mecanum_Send.data_check_num = (uint16_t)(tim14.ClockTime * 0.05);
	mecanum_Send.dead_flag = !check_robot_state.usart_state.GPIO_data;
	
	memcpy(pBuffer,&mecanum_Send,Mecanum_Datas_Send_Length);
	
	if(huart_x == &huart4 && (mecanum_recv_flag == 1 || mecanum_recv_count >= recv_count_max)){
		HAL_UART_Transmit_DMA(huart_x,Usart4_TxBuffer,Mecanum_Datas_Send_Length);
		mecanum_recv_flag = 0;
	}
}


void Mecanum_Datas_Modify(uint8_t* pdata, MecanumChassis_Recv* mecanum_recv){
	if(*(pdata) == 0xEE && *(pdata + 1) == 0xA2){
		uint8_t complish_flag;
		int8_t flags;
		
		complish_flag = (uint8_t)(*(pdata+2));
		flags = (int8_t)(*(pdata+3));
		
		mecanum_recv -> lift_complish_flag = complish_flag;
		mecanum_recv -> back_flag = flags;
	}
}

uint8_t mecanum_callback(uint8_t * recBuffer, uint16_t len){
	Mecanum_Datas_Modify(Usart4_RxBuffer, &mecanum_Recv);
	mecanum_recv_count = 0;
	mecanum_recv_flag = 1;
}