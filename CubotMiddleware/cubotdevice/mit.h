#ifndef _MIT_H
#define _MIT_H
#include <driver_can.h>
#include "stm32h7xx_hal.h"
#include "joint_control.h"



void ctrl_motor1(CAN_Object* hcan,DM_motor *Dm_motor, float _pos, float _vel, float _KP, float _KD, float _torq);
void ctrl_motor2(CAN_Object* hcan,DM_motor *Dm_motor, float _pos, float _vel);
void ctrl_motor3(CAN_Object* hcan,DM_motor *Dm_motor, float _vel);
void start_motor(CAN_Object* hcan,DM_motor *Dm_motor);
void lock_motor(CAN_Object* hcan,DM_motor *Dm_motor);
void DM_init(Manipulator_t* manipulator);
void DM_Restart(Manipulator_t* manipulator, int32_t clock, CAN_Object* hcan);

CAN_RxBuffer MitRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer);


#endif
