#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stm32h7xx_hal.h>
#include <driver_can.h>

#include "linux_list.h" 


#define K_ECD_TO_ANGLE 0.043945f  		//< 角度转换编码器刻度的系数：360/8192
#define ECD_RANGE_FOR_3508 8191				//< 编码器刻度值为0-8191
#define CURRENT_LIMIT_FOR_3508 16000   //< 控制电流范围为正负16384
#define ECD_RANGE_FOR_6020 8191				//< 编码器刻度值为0-8191
#define CURRENT_LIMIT_FOR_6020 29000   //< 控制电流范围为正负30000
#define ECD_RANGE_FOR_2006 8191				//< 编码器刻度值为0-8191
#define CURRENT_LIMIT_FOR_2006 10000   //< 控制电流范围为正负16384

typedef enum{
  Motor3508 = 0x00U,             //wu fu hao
	Motor6020 = 0x01U,
	Motor2006 = 0x02U,
  MIT       = 0x03U
}MotorType;

typedef struct{
	
  int16_t  Ecd;         	//< 当前编码器返回值
	int16_t  SpeedRPM;			//< 每分钟所转圈数
	int16_t  TorqueCurrent; //< 反馈力矩
	uint8_t  Temperature;	  //< 温度
	
	float  RawEcd;				//< 原始编码器数据
	int16_t  LastEcd;			  //< 上一时刻编码器返回值			
	float    Angle;					//< 解算后的编码器角度
	int16_t  AngleSpeed;	  //< 解算后的编码器角速度	
	int32_t  RoundCnt;			//< 累计转动圈数
	int32_t  TotalEcd;			//< 编码器累计增量值
	int32_t  TotalAngle;		//< 累计旋转角度

	float  Target;				//< 电机的期望参数
	int32_t  Output;  			//< 电机输出值，通常为电流和电压	
	float CanEcd[20] ;
	float CanAngleSpeed[20] ;
	float LvboAngle;
	int16_t  LvboEcd;
	int16_t  LvboSpeedRPM;
  int16_t  moter_id;
	
	//达妙电机相关数据
	float    MITangle;
	float    MITspeed;
	float    MITtorque;
}MotorData;


typedef struct 														
{
	uint8_t  CanNumber;			 										//< 电机所使用的CAN端口号
	uint16_t CanId;			 												//< 电机ID	
	uint8_t  MotorType;			 										//< 电机类型	
	uint16_t EcdOffset;	 									  		//< 电机初始零点
	uint16_t EcdFullRange;											//< 编码器量程
	int16_t  CurrentLimit;			 								//< 电调能承受的最大电流  
}MotorParam;

static  uint8_t CAN_update_data(MotorData* motor, CAN_RxBuffer rxBuffer);
typedef uint8_t (*Motor_DataUpdate)(MotorData* motor_data, CAN_RxBuffer rxBuffer); 

static  uint8_t CAN_fill_3508_2006_data(CAN_Object can, MotorData motor_data, uint16_t id);
typedef uint8_t (*CAN_FillMotorData)(CAN_Object can, MotorData motor_data, uint16_t id);

typedef struct {

    MotorData    Data;
	  MotorParam   Param;
	
	//list_t  
    list_t             list;
	  Motor_DataUpdate   MotorUpdate;											//< 更新电机运行数据的函数指针
	  CAN_FillMotorData  FillMotorData;	
}Motor;



static void MotorEcdtoAngle(Motor* motor);
static void MotorLvboEcdtoAngle(Motor* motor);
static void MotorOutputLimit(Motor* motor);
static  uint8_t CAN_fill_3508_2006_data(CAN_Object can, MotorData motor_data, uint16_t id);
static  uint8_t CAN_fill_6020_data( CAN_Object can, MotorData motor_data,uint16_t id);
static  uint8_t CAN_update_data(MotorData* motor, CAN_RxBuffer rxBuffer);
static void CAN_RegisteMotor(CAN_Object* canx, Motor* motor);
static void CAN_DeleteMotor(Motor* motor);
//int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
static  uint8_t CAN_MIT_update_data(MotorData* motor, CAN_RxBuffer rxBuffer);
void MotorInit(Motor* motor, uint16_t ecd_Offset, MotorType type, CanNumber canx, uint16_t id);
static Motor* MotorFind(uint16_t canid, CAN_Object canx);
void MotorRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer);
uint16_t MotorReturnID(Motor motor);
void MotorFillData(Motor* motor, int32_t output);
uint16_t MotorCanOutput(CAN_Object can, int16_t IDforTxBuffer);


	








	
	
	


#endif








