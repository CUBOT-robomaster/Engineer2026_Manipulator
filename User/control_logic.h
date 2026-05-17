#ifndef CONTROLLOGIC_H_
#define CONTROLLOGIC_H_

#include "hardware_config.h"
#include <check.h>
#include "joint_control.h"
#include "mit.h"
#include "mecanum_chassis.h"
#include <string.h>
#include "servo.h"
#include "auto.h"
#include "controllerl.h"


#define AtR 0.0174532f
#define RtA 57.296083f

void TIM14_Task(void);
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer);
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer);
void TIM14_Task_send(void);



//����Ϊ������ʱ����
extern float test01;
extern float test02;
extern uint16_t test03;
extern uint16_t test04;
extern uint16_t controller_count;

extern uint8_t Auto_state;
extern uint8_t Reset_flag;


#endif



