#include <tim.h>
#include <driver_timer.h>
#include "control_logic.h"
#include "stdio.h"
#include "modbus.h"
#include "protract.h"
#include "interaction.h"
#include "interaction_image.h"
#include "AIM_PDO.h"
#include "servo.h"
#include "controllerl.h"

CAN_RxBuffer Rxbuffer1,Rxbuffer2;
uint16_t controller_count = 0;

/* 以下为测试临时变量 */



void robot_offline_protection(){
	RobotOnlineState(&check_robot_state);
	check_robot_state.usart_state.GPIO_data = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	if(check_robot_state.usart_state.Check_receiver > 50 || check_robot_state.usart_state.GPIO_data == 0 || Auto_flags.motor_start_mode_flag == 1){
		rc_Ctrl.isOnline = 0;//遥控离线
	}
	else if(check_robot_state.usart_state.GPIO_data == 1 && Auto_flags.motor_start_mode_flag == 0){
		rc_Ctrl.isOnline = 1;
	}
	controller_count ++;
	if(controller_count < 450){
		Custom.image_recv.Coordinate.isonline = 1;
	}
	else{
		Custom.image_recv.Coordinate.isonline = 0;
	}
}
/**
  * @brief  主任务函数
  */
void TIM14_Task(void){
	tim14.ClockTime++;
	robot_offline_protection();
	if(tim14.ClockTime <= 2000){
		/* 达妙电机初始化 */
		DM_Restart(&Manipulator_Right, tim14.ClockTime, &can1);
		DM_Restart(&Manipulator_Left, tim14.ClockTime, &can2);

		if(tim14.ClockTime % 120 == 60){
			/* pitch轴先舵机初始化，防止堵转 */
			servo_move(hiwonder_Servo.pitch_servo.servo_id, hiwonder_Servo.pitch_servo.move_time, hiwonder_Servo.pitch_servo.position);		//475平视，275最低
		}
	}
	if(tim14.ClockTime > 2000 && tim14.ClockTime <= 4000){
		/* 8010电机初始化 */
		Motor_8010_Init(&Manipulator_Right, &Manipulator_Left);
		/* 初始化自定义控制器数据 */
		Customer_init_image_right(&Manipulator_Right, &Custom);
		Customer_init_image_left(&Manipulator_Left, &Custom);
		if(tim14.ClockTime % 120 == 0){
			/* yaw轴舵机初始化 */
			servo_move(hiwonder_Servo.yaw_servo.servo_id, hiwonder_Servo.yaw_servo.move_time, hiwonder_Servo.yaw_servo.position);			//950正前方，175正后方
		}
	}
	if(tim14.ClockTime > 4000){
		Auto_Control(&Manipulator_Right, &Manipulator_Left, &Auto_flags, &hiwonder_Servo, &Custom);
		joint_Ctrl(&Manipulator_Right, &Manipulator_Left);
		hiwonder_servo_control(tim14.ClockTime, &hiwonder_Servo);
		Usart8DmaPrintf("%f,%f,%f,%f\n",Manipulator_Right.Dm_8006_joint3.T,Manipulator_Right.joint3_deg.cc_recv,Custom.image_recv.Coordinate.joint6_left,Custom.image_recv.Coordinate.joint3_left);
		
		/* 上下板通信 */
		if(tim14.ClockTime % 7 == 0){
			Mecanum_data_Send(&huart4, Usart4_TxBuffer);
			mecanum_recv_count ++;
		}
	}
}

/**
  * @brief  CAN1接收中断回调
  */
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can1, (*rxBuffer)); 
	MitRxCallback(can1, (*rxBuffer));
	Rxbuffer1 = AIMRxCallback(can1, (*rxBuffer), &MOTOR_ONE);
	return 0;
}

/**
  * @brief  CAN2接收中断回调
  */
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can2, (*rxBuffer)); 	
	MitRxCallback(can2, (*rxBuffer));
	Rxbuffer2 = AIMRxCallback(can1, (*rxBuffer), &MOTOR_TWO);
	return 0;
}
