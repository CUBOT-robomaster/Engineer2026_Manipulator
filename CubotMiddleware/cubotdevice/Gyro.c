/**@file Gyro.c
* @brief    板级支持包，串口管理器配置文件，用户回调重定义
* @details  主要包括构建串口管理器，提供串口初始化和用户回调重定义
* @author      RyanJiao  any question please send mail to 1095981200@qq.com

* @date        2021-8-23
* @version     V1.1
* @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
**********************************************************************************
* @attention
* 硬件平台: STM32H750VBT \n
* SDK版本：-++++
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>创建初始版本
* </table>
*
**********************************************************************************
*/

/**********************************************************************************
 ==============================================================================
						  How to use this driver
 ==============================================================================

	添加 MPU6050.h

	1. 调用MPU6050_Init()
  2. 把 Gyro_Get_Data() 放在1ms的中断里面
  

  ********************************************************************************/


#include "Gyro.h"
#include "mpu6050.h"
#include "driver_timer.h"
#include "kalman.h"
#include "holder.h"
#include "hardware_config.h"
#include "filter.h"
#include "attitude.h"
MpuDebug mpudebug =
{
	.mpuDebugTime=0,
	.DebugBeginFlag=0,
	.DebugFinishFlag=0 
};

/**
  *@brife  usart5接受串口陀螺仪
	*/
uint8_t Usart5_RxBuffer[30]__attribute__((at(0x24002100)));
uint8_t Usart5_TxBuffer[30]__attribute__((at(0x24002130)));

UART_RxBuffer uart5_buffer={
	.Data = Usart5_RxBuffer,
	.Size = 50
};


gyro_data_t  gyro_data=
{
	.sens_yaw  =0.499f,  // yaw   角速度积分为角度的值  内置模块使用
	.sens_pitch=0.6783,  // pitch 角速度积分为角度的值  未用
  .cqhlp = 10
};

gyro_data_t  gyro_data2=
{
	.sens_yaw  =0.3580,  // yaw   角速度积分为角度的值   未用
	.sens_pitch=0.2951,  // pitch 角速度积分为角度的值   外置模块使用
  .cqhlp = 10
};

void MPU_Init(void)
{
	//MUP structure variable define
	MPU_Region_InitTypeDef MPU_Config;
	
	/*-----------Open FPU--------*///High speed FLOAT calculate
	SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
	/*-----------Open Cache------------*/
	SCB_EnableICache();//使能I-Cache
  SCB_EnableDCache();//使能D-Cache   
	SCB->CACR|=1<<2;   //强制D-Cache透写,如不开启,实际使用中可能遇到各种问题	
	/*-----------Open MPU------------*/
	HAL_MPU_Disable();
	
	MPU_Config.Enable=MPU_REGION_ENABLE;
	MPU_Config.Number=MPU_REGION_NUMBER1;//保护区编号 1
	MPU_Config.BaseAddress= 0x24000000;//保护区基地址
	MPU_Config.Size=MPU_REGION_SIZE_512KB;//设置保护区512k
	MPU_Config.SubRegionDisable=0x00;//禁止子区域
	MPU_Config.TypeExtField=MPU_TEX_LEVEL0;//设置类型扩展域为level0
	MPU_Config.AccessPermission=MPU_REGION_FULL_ACCESS;//全访问（特权&用户都可访问）
	MPU_Config.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;//允许指令访问
	MPU_Config.IsShareable=MPU_ACCESS_SHAREABLE;//允许共享
	MPU_Config.IsCacheable=MPU_ACCESS_CACHEABLE;//允许cache
	MPU_Config.IsBufferable=MPU_ACCESS_NOT_BUFFERABLE;//不允许缓冲 DMA模式下要设为不允许
	HAL_MPU_ConfigRegion(&MPU_Config);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}	


void Gyro_Init(void)
{
	//>上电
		MPU6050_PowerOn();
	//>初始化
	  //MPU6050_Init();
		MPU6050_Init2();
	//>计数为 0
	  gyro_data.cnt=0;
		gyro_data2.cnt=0;

}


void Gyro_Reset(void)
{
	//>下电
	  MPU6050_PowerOff();
	  HAL_Delay(50);
	
	//>上电初始化
    Gyro_Init();

}

/*
 *测速数据 
 *yaw静态角速度   
 *pitch静态角速度 
 */

//>放在1ms的定时器中断中
float yaw_data_filter_mpu[5]={0};
float pitch_data_filter_mpu[5]={0};
int8_t yaw_filter_mpu_cnt=0;
int8_t pitch_filter_mpu_cnt=0;
void Gyro_Get_Data(void){
	//>数据更新次数累加
  gyro_data.cnt++;
	
  //>MPU6050数据更新
	MPU6050_RawDataUpdate(&mpuAngle,1);

if(gyro_data.cnt>1000){	
		//>四元数解算出 pitch角度
	
	//>角度化为弧度  加速度进行卡尔曼滤波  （有一处修改）
	PrepareForIMU(&sensor,&mpuAngle);
	IMUupdate(&sensor,&mpuAngle,1);
	
	gyro_data.gyro_w_yaw   = mpuAngle.gyroRaw.z-(int16_t)sensor.gyro.quiet.z;
	gyro_data.gyro_w_pitch = mpuAngle.gyroRaw.x-(int16_t)sensor.gyro.quiet.x;

	if(abs(gyro_data.gyro_w_yaw)>100){
			gyro_data.yaw_speed=((gyro_data.gyro_w_yaw)*0.0001*gyro_data.sens_yaw);
			gyro_data.yaw_angle= gyro_data.yaw_angle + gyro_data.yaw_speed;}
	
	if(abs(gyro_data.gyro_w_pitch)>100){		
			gyro_data.pitch_speed = ((gyro_data.gyro_w_pitch)*0.0001f*gyro_data.sens_pitch);
	    gyro_data.pitch_angle= gyro_data.pitch_angle + gyro_data.pitch_speed;}
		}
}

void Gyro_Get_Data2(void){
	//>数据更新次数累加
  gyro_data2.cnt++;
	
  //>MPU6050数据更新
	MPU6050_RawDataUpdate(&mpuAngle2,2);

if(gyro_data2.cnt>1000){	
	
	//>角度化为弧度  加速度进行卡尔曼滤波  （有一处修改）
	PrepareForIMU(&sensor2,&mpuAngle2);
	//>四元数解算出 pitch角度
	IMUupdate(&sensor2,&mpuAngle2,2);	
	
	gyro_data2.gyro_w_yaw   = mpuAngle2.gyroRaw.z-(int16_t)sensor2.gyro.quiet.z;
	gyro_data2.gyro_w_pitch = mpuAngle2.gyroRaw.x-(int16_t)sensor2.gyro.quiet.x;

	if(abs(gyro_data2.gyro_w_yaw)>10){
			gyro_data2.yaw_speed=((gyro_data2.gyro_w_yaw)*0.0001*gyro_data2.sens_yaw);
			gyro_data2.yaw_angle= gyro_data2.yaw_angle + gyro_data2.yaw_speed;}
		
	if(abs(gyro_data2.gyro_w_pitch)>10){		
			gyro_data2.pitch_speed = ((gyro_data2.gyro_w_pitch)*0.0001f*gyro_data2.sens_pitch);
	    gyro_data2.pitch_angle= gyro_data2.pitch_angle + gyro_data2.pitch_speed;}
		}
}

void MPU_Get_Data(Holder_t* holder)
{
	//Gyro_Get_Data();  //板载陀螺仪
	Gyro_Get_Data2();   //外置IIC陀螺仪模块
	holder->Yaw.MPU6050_Angle=-gyro_data2.yaw_angle;
	holder->Yaw.MPU6050_Angle_speed=gyro_data2.yaw_speed;
  holder->Yaw.MPU6050_Angle_speed1=holder->Yaw.MPU6050_Angle_speed*150;
//holder->Yaw.MPU6050_Angle_speed1=LPFilter(holder->Yaw.MPU6050_Angle_speed1 ,&LPF_yaw_mpu);  //一阶低通滤波
	holder->Pitch.MPU6050_Angle=gyro_data2.pitch_angle;
	holder->Pitch.MPU6050_Angle_speed=gyro_data2.pitch_speed;
	holder->Pitch.MPU6050_Angle_speed1=holder->Pitch.MPU6050_Angle_speed*150;
//holder->Pitch.MPU6050_Angle_speed1=LPFilter(holder->Pitch.MPU6050_Angle_speed1 ,&LPF_pitch_mpu);   //一阶低通滤波
	
	
}

/**
  * @brief   串口5串口陀螺仪回调函数 
  * @param[in]  
  */
//uint8_t Gyro_callback(uint8_t * recBuffer, uint16_t len)
//{
////	angle_deal(recBuffer,&Holder);
//	HWT906_Data_Deal(recBuffer,&Holder);
//	return 0;
//}

///**
//  * @brief   正点原子串口陀螺仪回调函数 ,波特率115200
//  * @param[in]  
//  */
//void angle_deal(uint8_t *angle_receive,Attitude_t *attitude)
//{
//	if(angle_receive[0]==0x55 && angle_receive[1]==0x55){
//		if(angle_receive[15]==0x03){
//			attitude->Pitch = -1*(float) ( (int16_t) (angle_receive[26]<<8) | angle_receive[25]) / 32768 * 2000;// * 0.01745;
//			attitude->Yaw.Uart6050_Angle_speed = -1*(float) ( (int16_t) (angle_receive[28]<<8) | angle_receive[27]) / 32768 * 2000;// * 0.01745;
//		}
//		if(angle_receive[2]==0x02){
//			attitude->q0 = (float)((int16_t)(angle_receive[5]<<8 | angle_receive[4])) / 32768;
//			attitude->q1 = (float)((int16_t)(angle_receive[7]<<8 | angle_receive[6])) / 32768;
//			attitude->q2 = (float)((int16_t)(angle_receive[9]<<8 | angle_receive[8])) / 32768;
//			attitude->q3 = (float)((int16_t)(angle_receive[11]<<8 | angle_receive[10])) / 32768;
//		}
//  		attitude->Yaw.Uart6050_Angle_raw = -atan2(2 * holder->q1 * holder->q2 + 2 * holder->q0* holder->q3, -2 * holder->q2*holder->q2 - 2 * holder->q3 * holder->q3 + 1)*RtA; 
//      attitude->Pitch.Uart6050_Angle_raw = -asin(-2 * holder->q1 * holder->q3 + 2 * holder->q0 * holder->q2)*RtA;
//		}
//	}

//	
///**
//  * @brief   维特串口陀螺仪回调函数，波特率915200
//  * @param[in]  
//  */	
//float HWT906_pitch;
//float HWT906_yaw;
//uint16_t count=0;
//float count1=0;
//void HWT906_Data_Deal(uint8_t *data_receive,Holder_t* holder)
//{
//	count++;
//	if(tim14.ClockTime%1000==0)
//	{
//		count1=count;
//		count=0;
//	}
//	if(data_receive[0]==0x55 && data_receive[1]==0x51){
//		holder->Pitch.Uart6050_Angle_acc = (float)((int16_t)(data_receive[3]<<8|data_receive[2]))*32768*16*9.8f;
//		holder->Yaw.Uart6050_Angle_acc = (float)((int16_t)(data_receive[7]<<8|data_receive[6]))*32768*16*9.8f;
//	}
//	if(data_receive[11]==0x55 && data_receive[12]==0x52){
//		holder->Pitch.Uart6050_Angle_speed = (float)((int16_t)(data_receive[14]<<8|data_receive[13]))/32768*2000;
//		holder->Yaw.Uart6050_Angle_speed = (float)((int16_t)(data_receive[18]<<8|data_receive[17]))/32768*2000;
//	}
//	if(data_receive[22]==0x55 && data_receive[23]==0x59){
//		holder->q0 = (float)((int16_t)(data_receive[25]<<8|data_receive[24]))/32768;
//		holder->q1 = (float)((int16_t)(data_receive[27]<<8|data_receive[26]))/32768;
//		holder->q2 = (float)((int16_t)(data_receive[29]<<8|data_receive[28]))/32768;
//		holder->q3 = (float)((int16_t)(data_receive[31]<<8|data_receive[30]))/32768;
//	}
//  	holder->Yaw.Uart6050_Angle_raw = -atan2(2 * holder->q1 * holder->q2 + 2 * holder->q0* holder->q3, -2 * holder->q2*holder->q2 - 2 * holder->q3 * holder->q3 + 1)*RtA; 
//		holder->Pitch.Uart6050_Angle_raw = atan2(2 * holder->q2 * holder->q3 + 2 * holder->q0 * holder->q1, -2 * holder->q1 * holder->q1 - 2 * holder->q2* holder->q2 + 1)* RtA;    // 本为roll轴角四元数解算公式，因为陀螺仪固定方向的原因，将roll角当做pitch角用
////	  holder->Pitch.Uart6050_Angle_raw = -asin(-2 * holder->q1 * holder->q3 + 2 * holder->q0 * holder->q2)*RtA;

//}

//float sens_pitch_change = 1;
///*将串口陀螺仪接收到的-180到180的角度转换成累加式的角度*/
//void HolderAngleChange(Holder_t *holder)
//{
//	if(holder->Pitch.MPU6050_Angle_Change!=0)
//	sens_pitch_change=abs(holder->Pitch.Can_Angle_Change) /abs(holder->Pitch.MPU6050_Angle_Change);

//  if(holder->Yaw.Uart6050_Angle<170 && holder->Yaw.Uart6050_Angle >-170)
//		 holder->Yaw.Uart6050_Angle = holder->Yaw.Uart6050_Angle_raw;
//	else{
//		if(holder->Yaw.Uart6050_Angle_raw<-140 && holder->Yaw.Uart6050_Angle_raw_last>140)
//			holder->Yaw.Uart6050_Angle+=(360+holder->Yaw.Uart6050_Angle_raw - holder->Yaw.Uart6050_Angle_raw_last);
//		else if(holder->Yaw.Uart6050_Angle_raw>140 && holder->Yaw.Uart6050_Angle_raw_last<-140)
//			holder->Yaw.Uart6050_Angle+=(360-holder->Yaw.Uart6050_Angle_raw + holder->Yaw.Uart6050_Angle_raw_last);
//		else
//			holder->Yaw.Uart6050_Angle += (holder->Yaw.Uart6050_Angle_raw - holder->Yaw.Uart6050_Angle_raw_last);
//    	}
//		holder->Yaw.Uart6050_Angle_raw_last=Holder.Yaw.Uart6050_Angle_raw;
//  	holder->Pitch.Uart6050_Angle = Holder.Pitch.Uart6050_Angle_raw; 
//	  if(tim14.ClockTime%1000 == 0)
//		{			
//		holder->Pitch.Can_Angle_Change = Holder.Pitch.Last_Can_Angle - Holder.Pitch.Can_Angle;
//		holder->Pitch.MPU6050_Angle_Change = Holder.Pitch.Last_MPU6050_Angle - Holder.Pitch.MPU6050_Angle;
//		holder->Pitch.Last_Can_Angle = Holder.Pitch.Can_Angle;
//		holder->Pitch.Last_MPU6050_Angle = Holder.Pitch.MPU6050_Angle;
//		}
//}

///**
//  * @brief  校准陀螺仪角度
//  */
//uint8_t Mpu6050_senceDebug(MpuDebug* mpuDebug,gyro_data_t* gyro_data)
//{
//if((tim14.ClockTime>1000)&&(mpuDebug->DebugFinishFlag==0)&&(mpuDebug->DebugBeginFlag==1))
//{
//	mpuDebug->mpuDebugTime++;
//	if(mpuDebug->mpuDebugTime==200)
//	{
//		mpuDebug->Yaw.MpuAngle1=Holder.Yaw .MPU6050_Angle;
//		mpuDebug->Yaw.CanAngle1=Holder.Yaw.Can_Angle;
//		mpuDebug->Pitch.MpuAngle1=Holder.Pitch .MPU6050_Angle;
//		mpuDebug->Pitch.CanAngle1=Holder.Pitch.Can_Angle;
//	}
//	if(mpuDebug->mpuDebugTime==400)
//	{
//		mpuDebug->Yaw.MpuAngle2=Holder.Yaw .MPU6050_Angle;
//		mpuDebug->Yaw.CanAngle2=Holder.Yaw.Can_Angle;
//		mpuDebug->Pitch.MpuAngle2=Holder.Pitch .MPU6050_Angle;
//		mpuDebug->Pitch.CanAngle2=Holder.Pitch.Can_Angle;
//	}
//	if(mpuDebug->mpuDebugTime==600)
//	{
//		mpuDebug->Yaw.MpuAngle3=Holder.Yaw .MPU6050_Angle;
//		mpuDebug->Yaw.CanAngle3=Holder.Yaw.Can_Angle;
//		mpuDebug->Pitch.MpuAngle3=Holder.Pitch .MPU6050_Angle;
//		mpuDebug->Pitch.CanAngle3=Holder.Pitch.Can_Angle;
//		mpuDebug->Yaw.MpuAngleChange=((mpuDebug->Yaw.MpuAngle3-mpuDebug->Yaw.MpuAngle2)+(mpuDebug->Yaw.MpuAngle2-mpuDebug->Yaw.MpuAngle1))*0.5;
//		mpuDebug->Yaw.CanAngleChange=((mpuDebug->Yaw.CanAngle3-mpuDebug->Yaw.CanAngle2)+(mpuDebug->Yaw.CanAngle2-mpuDebug->Yaw.CanAngle1))*0.5;
//		mpuDebug->Pitch.MpuAngleChange=((mpuDebug->Pitch.MpuAngle3-mpuDebug->Pitch.MpuAngle2)+(mpuDebug->Pitch.MpuAngle2-mpuDebug->Pitch.MpuAngle1))*0.5;
//		mpuDebug->Pitch.CanAngleChange=((mpuDebug->Pitch.CanAngle3-mpuDebug->Pitch.CanAngle2)+(mpuDebug->Pitch.CanAngle2-mpuDebug->Pitch.CanAngle1))*0.5;
//    mpuDebug->Yaw.SenceBili=abs(mpuDebug->Yaw.CanAngleChange)/abs(mpuDebug->Yaw.MpuAngleChange);
//		mpuDebug->Pitch.SenceBili=abs(mpuDebug->Pitch.CanAngleChange)/abs(mpuDebug->Pitch.MpuAngleChange);
//		if((mpuDebug->Yaw.SenceBili>=2)||(mpuDebug->Yaw.SenceBili<=0))
//			mpuDebug->Yaw.SenceBili=1;
//		if((mpuDebug->Pitch.SenceBili>=2)||(mpuDebug->Pitch.SenceBili<=0))
//			mpuDebug->Pitch.SenceBili=1;
//		gyro_data->sens_yaw=gyro_data->sens_yaw*(mpuDebug->Yaw.SenceBili);
//		gyro_data->sens_pitch=gyro_data->sens_pitch*(mpuDebug->Pitch.SenceBili);
//	}
//	if(mpuDebug->mpuDebugTime>600)
//	{
//		mpuDebug->mpuDebugTime=600;
//	  mpuDebug->DebugFinishFlag=1;
//	}
// }
//}

