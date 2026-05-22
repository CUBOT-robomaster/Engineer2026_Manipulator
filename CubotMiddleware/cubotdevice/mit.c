#include "mit.h"
#include "filter.h"
#include "joint_control.h"
#include "control_logic.h"
#include "stm32h7xx_hal.h"
#include <driver_can.h>
#include <motor.h>

#define P_MIN  -12.5f
#define P_MAX   12.5f
#define V_MIN  -30.0f
#define V_MAX   30.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f   //Kp 的范围为[0,500]
#define KD_MIN  0.0f     //Kd 的范围为[0,5]
#define KD_MAX  5.0f
#define T_MIN   -10.0f
#define T_MAX   10.0f

CAN_TxBuffer txbuf_MODE1;
CAN_TxBuffer txbuf_MODE2;
CAN_TxBuffermit txbuf_MODE3;
CAN_TxBuffer txbuf_start;
CAN_TxBuffer txbuf_lock;
CAN_TxBuffer txbuf_clean;


void DM_init(Manipulator_t* manipulator){
	/*joint1*/
		manipulator -> joint1_deg.rad = manipulator -> Dm_4340_joint1.pos;
		manipulator -> joint1_deg.rad_init = manipulator -> joint1_deg.rad;
		manipulator -> joint1_deg.angle = manipulator -> Dm_4340_joint1.pos * RtA;
		manipulator -> joint1_deg.angle_init = manipulator -> Dm_4340_joint1.pos * RtA;
	
	/*joint2*/
		manipulator -> joint2_deg.rad = manipulator -> Dm_4340_joint2.pos;
		manipulator -> joint2_deg.rad_init = manipulator -> joint2_deg.rad;
		manipulator -> joint2_deg.angle = manipulator -> Dm_4340_joint2.pos * RtA;
		manipulator -> joint2_deg.angle_init = manipulator -> Dm_4340_joint2.pos * RtA;

	/*joint3*/
		manipulator -> joint3_deg.rad = manipulator -> Dm_8006_joint3.pos;
		manipulator -> joint3_deg.rad_init = manipulator -> joint3_deg.rad;
		manipulator -> joint3_deg.angle = manipulator -> Dm_8006_joint3.pos * RtA;
		manipulator -> joint3_deg.angle_init = manipulator -> Dm_8006_joint3.pos * RtA;
	
	/*joint4*/
		manipulator -> joint4_deg.rad = manipulator -> Dm_4310_joint4.pos;
		manipulator -> joint4_deg.rad_init = manipulator -> joint4_deg.rad;
		manipulator -> joint4_deg.angle = manipulator -> Dm_4310_joint4.pos * RtA;
		manipulator -> joint4_deg.angle_init = manipulator -> Dm_4310_joint4.pos * RtA;
	
	/*joint5*/
		manipulator -> joint5_deg.rad = manipulator -> Dm_4310_joint5.pos;
		manipulator -> joint5_deg.rad_init = manipulator -> joint5_deg.rad;
		manipulator -> joint5_deg.angle = manipulator -> Dm_4310_joint5.pos * RtA;
		manipulator -> joint5_deg.angle_init = manipulator -> Dm_4310_joint5.pos * RtA;

	/*joint6*/
		manipulator -> joint6_deg.rad = manipulator -> Dm_4310_joint6.pos;
		manipulator -> joint6_deg.rad_init = manipulator -> joint6_deg.rad;
		manipulator -> joint6_deg.angle = manipulator -> Dm_4310_joint6.pos * RtA;
		manipulator -> joint6_deg.angle_init = manipulator -> Dm_4310_joint6.pos * RtA;
}
	
void DM_Restart(Manipulator_t* manipulator, int32_t clock, CAN_Object* hcan){
	if(clock % 6 == 0){
		start_motor(hcan, &manipulator -> Dm_4340_joint1);
	}
	if(clock % 6 == 1){
		start_motor(hcan, &manipulator -> Dm_4340_joint2);
	}
	if(clock % 6 == 2){
		start_motor(hcan, &manipulator -> Dm_8006_joint3);
	}
	if(clock % 6 == 3){
		start_motor(hcan, &manipulator -> Dm_4310_joint4);
	}
	if(clock % 6 == 4){
		start_motor(hcan, &manipulator -> Dm_4310_joint5);
	}
	if(clock % 6 == 5){
		start_motor(hcan, &manipulator -> Dm_4310_joint6);
	}
	DM_init(manipulator);//使能达妙电机并初始化
}

void DM_Error_clean(Manipulator_t *manipulator, int32_t clock, CAN_Object *hcan){
	if(clock % 6 == 0){
		Error_clean(hcan, &manipulator -> Dm_4340_joint1);
	}
	if(clock % 6 == 1){
		Error_clean(hcan, &manipulator -> Dm_4340_joint2);
	}
	if(clock % 6 == 2){
		Error_clean(hcan, &manipulator -> Dm_8006_joint3);
	}
	if(clock % 6 == 3){
		Error_clean(hcan, &manipulator -> Dm_4310_joint4);
	}
	if(clock % 6 == 4){
		Error_clean(hcan, &manipulator -> Dm_4310_joint5);
	}
	if(clock % 6 == 5){
		Error_clean(hcan, &manipulator -> Dm_4310_joint6);
	}
}

int float_to_uint(float x, float x_min, float x_max, int bits)
	{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
  }
    
float uint_to_float(int x_int, float x_min, float x_max, int bits)
	{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
  }  
  
  
  

void ctrl_motor1(CAN_Object* hcan,DM_motor *Dm_motor, float _pos, float _vel, float _KP, float _KD, float _torq){
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
	txbuf_MODE1.Identifier = Dm_motor->CAN_ID;
	txbuf_MODE1.Data[0]= (pos_tmp >> 8);
	txbuf_MODE1.Data[1] = pos_tmp;
	txbuf_MODE1.Data[2] = (vel_tmp >> 4);
	txbuf_MODE1.Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	txbuf_MODE1.Data[4] = kp_tmp;
	txbuf_MODE1.Data[5] = (kd_tmp >> 4);
	txbuf_MODE1.Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	txbuf_MODE1.Data[7] = tor_tmp;

  CAN_Send(hcan, &txbuf_MODE1);

}
void ctrl_motor2(CAN_Object* hcan,DM_motor *Dm_motor, float _pos, float _vel)
{
	uint8_t *pbuf,*vbuf;
	pbuf=(uint8_t*)&_pos;
	vbuf=(uint8_t*)&_vel;
	
    txbuf_MODE2.Identifier=Dm_motor->CAN_ID;
	txbuf_MODE2.Data[0]  = *pbuf;
	txbuf_MODE2.Data[1]  = *(pbuf+1);
	txbuf_MODE2.Data[2]  = *(pbuf+2);
	txbuf_MODE2.Data[3]  = *(pbuf+3);
	txbuf_MODE2.Data[4]  = *vbuf;
	txbuf_MODE2.Data[5]  = *(vbuf+1);
	txbuf_MODE2.Data[6]  = *(vbuf+2);
	txbuf_MODE2.Data[7]  = *(vbuf+3);
	

	CAN_Send(hcan, &txbuf_MODE2);
}	
void start_motor(CAN_Object* hcan,DM_motor *Dm_motor)
{
	txbuf_start.Identifier = Dm_motor->CAN_ID;
	txbuf_start.Data[0] = 0xFF;
	txbuf_start.Data[1] = 0xFF;
	txbuf_start.Data[2] = 0xFF;
	txbuf_start.Data[3] = 0xFF;
	txbuf_start.Data[4] = 0xFF;
	txbuf_start.Data[5] = 0xFF;
	txbuf_start.Data[6] = 0xFF;
	txbuf_start.Data[7] = 0xFC;
	
	CAN_Send(hcan, &txbuf_start);
}	

void lock_motor(CAN_Object* hcan,DM_motor *Dm_motor)
{
	txbuf_lock.Identifier = Dm_motor->CAN_ID;
	txbuf_lock.Data[0] = 0xFF;
	txbuf_lock.Data[1] = 0xFF;
	txbuf_lock.Data[2] = 0xFF;
	txbuf_lock.Data[3] = 0xFF;
	txbuf_lock.Data[4] = 0xFF;
	txbuf_lock.Data[5] = 0xFF;
	txbuf_lock.Data[6] = 0xFF;
	txbuf_lock.Data[7] = 0xFD;
	
	CAN_Send(hcan, &txbuf_lock);
}		

void Error_clean(CAN_Object* hcan,DM_motor *Dm_motor){
	txbuf_clean.Identifier = Dm_motor->CAN_ID;
	txbuf_clean.Data[0] = 0xFF;
	txbuf_clean.Data[1] = 0xFF;
	txbuf_clean.Data[2] = 0xFF;
	txbuf_clean.Data[3] = 0xFF;
	txbuf_clean.Data[4] = 0xFF;
	txbuf_clean.Data[5] = 0xFF;
	txbuf_clean.Data[6] = 0xFF;
	txbuf_clean.Data[7] = 0xFB;
	
	CAN_Send(hcan, &txbuf_clean);
}

void DM_data(DM_motor *Dm_motor,unsigned char* pBuffer)
{
	uint16_t buffer_p;
	uint16_t buffer_v;
	uint16_t buffer_t;
    Dm_motor->id=pBuffer[0]&0x0f;
	Dm_motor->Error=pBuffer[0]>>4;
	buffer_p=pBuffer[2]|(pBuffer[1]<<8);
	Dm_motor->pos=uint_to_float(buffer_p, P_MIN, P_MAX, 16);
	buffer_v=(pBuffer[4]|(pBuffer[3]<<8))>>4;
	Dm_motor->vel=uint_to_float(buffer_v,V_MIN, V_MAX, 12);
	buffer_t=pBuffer[5]|((pBuffer[4]&0x0f)<<8);
	Dm_motor->T=uint_to_float(buffer_t,T_MIN, T_MAX, 12);
	Dm_motor->T_mos=pBuffer[6];
	Dm_motor->T_Rotor=pBuffer[7];
	

}
CAN_RxBuffer MitRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer)
{
	uint32_t master_id;	
	master_id=rxBuffer.Header.Identifier;
		switch(master_id)
		{
			case 0x01:
			{
			   DM_data(&Manipulator_Right.Dm_4340_joint2,rxBuffer.Data);
				break;
			}
			case 0x02:
			{
			   DM_data(&Manipulator_Right.Dm_4340_joint1,rxBuffer.Data);
				break;
			}
			case 0x03:
			{
				DM_data(&Manipulator_Right.Dm_8006_joint3,rxBuffer.Data);
				break;
			}
			case 0x04:
			{
				DM_data(&Manipulator_Right.Dm_4310_joint4,rxBuffer.Data);
				break;
			}
			case 0x05:
			{
				DM_data(&Manipulator_Right.Dm_4310_joint5,rxBuffer.Data);
				break;
			}
			case 0x06:
			{
				DM_data(&Manipulator_Left.Dm_4310_joint6,rxBuffer.Data);
				break;
			}
			
			case 0x11:
			{
			   DM_data(&Manipulator_Left.Dm_4340_joint1,rxBuffer.Data);
				break;
			}
			case 0x12:
			{
			   DM_data(&Manipulator_Left.Dm_4340_joint2,rxBuffer.Data);
				break;
			}
			case 0x13:
			{
				DM_data(&Manipulator_Left.Dm_8006_joint3,rxBuffer.Data);
				break;
			}
			case 0x14:
			{
				DM_data(&Manipulator_Left.Dm_4310_joint4,rxBuffer.Data);
				break;
			}
			case 0x15:
			{
				DM_data(&Manipulator_Left.Dm_4310_joint5,rxBuffer.Data);
				break;
			}
			case 0x16:
			{
				DM_data(&Manipulator_Right.Dm_4310_joint6,rxBuffer.Data);
				break;
			}
			
		   default:   ;
		}	

	return rxBuffer;
}