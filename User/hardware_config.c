#include "hardware_config.h"
#include <dr16.h>
#include <driver_can.h>
#include <control_logic.h>
#include <driver_timer.h>
#include "controllerl.h"
#include "interaction_image.h"
#include "vt13.h"

Attitude_t attiTude;
RC_Ctrl RC_Ctl;

Motors motor6020;
Motors motor3508;
void HardwareConfig(){
	/*遥控测试代码*/
	DR16Init(&rc_Ctrl);
	VT13Init(&vT13);
	UARTx_Init(&huart1,DR16_callback);//DR16_callback
	UART_ENABLE_IT(&uart1,&uart1_buffer);
	UART_Receive_DMA(&uart1, &uart1_buffer);
	
	/*遥控测试代码*/
	UARTx_Init(&huart2, imagetrans_callback);	//图传接收串口回调
	UART_ENABLE_IT(&uart2, &uart2_buffer);
	UART_Receive_DMA(&uart2, &uart2_buffer);

	/* 新板子没有串口3 */
	// UARTx_Init(&huart3,Gyro_callback);
	// UART_ENABLE_IT(&uart3,&uart3_buffer);
	// UART_Receive_DMA(&uart3, &uart3_buffer); 

	UARTx_Init(&huart4,mecanum_callback);
	UART_ENABLE_IT(&uart4,&uart4_buffer);
	UART_Receive_DMA(&uart4, &uart4_buffer); 

	UARTx_Init(&huart5,Gyro_callback);  //Referee_callback,Brain_callback
	UART_ENABLE_IT(&uart5,&uart5_buffer);
	UART_Receive_DMA(&uart5, &uart5_buffer);

	UARTx_Init(&huart6,Gyro_callback);  //Referee_callback,Brain_callback
	UART_ENABLE_IT(&uart6,&uart6_buffer);
	UART_Receive_DMA(&uart6, &uart6_buffer);
	UARTx_Init(&huart7,Gyro_callback);
	UART_ENABLE_IT(&uart7,&uart7_buffer);
	UART_Receive_DMA(&uart7, &uart7_buffer);


	UARTx_Init(&huart8,Gyro_callback);  //
	UART_ENABLE_IT(&uart8,&uart8_buffer);
	UART_Receive_DMA(&uart8, &uart8_buffer);
	/*串口测试需要打开*/

	CANx_Init(&hfdcan1, CAN1_rxCallBack);
	CAN_Open(&can1);

	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open(&can2);

	TIMx_Init(&htim14, TIM14_Task);
	TIM_Open(&tim14);
}


