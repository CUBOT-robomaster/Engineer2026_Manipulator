#include "attitude.h"
#include "servo.h"
#include <dr16.h>
#include <math.h>
#include <driver_timer.h>
#include "driver_usart.h"
#include "hardware_config.h"
#include "math.h"
#include "mit.h"
#include <ore.h>
#include "control_logic.h"
#define abs(x) ((x)>0? (x):(-(x)))

uint16_t timer=0;
int16_t error3508;
uint16_t rscount1_3508=0;
uint16_t rscount2_3508=0;
float pitchangel;
float pitch_sense = 0.5;

uint8_t hwt_rec[10]={0};
UART_RxBuffer uart7_buffer={
		.Data = hwt_rec,
		.Size = 10
	};
//UART_RxBuffer uart2_buffer={
//		.Data = hwt_rec,
//		.Size = 10
//	};
//UART_RxBuffer uart3_buffer={
//		.Data = hwt_rec,
//		.Size = 10
//	};
//UART_RxBuffer uart4_buffer={
//		.Data = hwt_rec,
//		.Size = 10
//	};
UART_RxBuffer uart5_buffer={
		.Data = hwt_rec,
		.Size = 10
	};
UART_RxBuffer uart6_buffer={
		.Data = hwt_rec,
		.Size = 10
	};
UART_RxBuffer uart8_buffer={
		.Data = hwt_rec,
		.Size = 10
	};

///**
//  * @brief  转塔初始化
//  */
//void Attitude_Init(Attitude_t *attitude, BasePID_Object roll_speed_pid, BasePID_Object roll_angle_pid, BasePID_Object pitch_speed, BasePID_Object pitch_angle, CanNumber canx)
//{
//		attitude->Motors_DM.pos_yaw = 1.33;//1.33，1.8
//		MotorInit(&attitude->Motors2006.motor[0], 0, Motor2006, canx , 0x201);
//    MotorInit(&attitude->Motors6020.motor[0], 311, Motor3508, canx , 0x205);		//转塔pitch轴
//		BasePID_Init(&attitude->Motors2006.RunPID[0], roll_speed_pid.Kp, roll_speed_pid.Ki, roll_speed_pid.Kd, roll_speed_pid.KiPartDetachment);
//		BasePID_Init(&attitude->Motors2006.RunPID[1], roll_angle_pid.Kp, roll_angle_pid.Ki, roll_angle_pid.Kd, roll_angle_pid.KiPartDetachment);
//	  BasePID_Init(&attitude->Motors6020.pitchPID[0], pitch_speed.Kp, pitch_speed.Ki, pitch_speed.Kd, pitch_speed.KiPartDetachment);
//		BasePID_Init(&attitude->Motors6020.pitchPID[1], pitch_angle.Kp, pitch_angle.Ki, pitch_angle.Kd, pitch_angle.KiPartDetachment);
//}	

//void Attitude_6020Init(Attitude_t *attitude, BasePID_Object yaw_angle_pid ,BasePID_Object yaw_speed_pid, BasePID_Object yaw2_angle_pid ,BasePID_Object yaw2_speed_pid, CanNumber canx)
//{
//		MotorInit(&attitude->Motors6020.motor[0],  203, Motor6020, canx , 0x207);
//	    MotorInit(&attitude->Motors6020.motor[1],  393, Motor6020, canx , 0x206);
//		BasePID_Init(&attitude->Motors6020.turnPID[0], yaw_angle_pid.Kp, yaw_angle_pid.Ki, yaw_angle_pid.Kd, yaw_angle_pid.KiPartDetachment);
//	    BasePID_Init(&attitude->Motors6020.turnPID[1], yaw_speed_pid.Kp, yaw_speed_pid.Ki, yaw_speed_pid.Kd, yaw_speed_pid.KiPartDetachment);
//	    BasePID_Init(&attitude->Motors6020.turnPID[2], yaw2_angle_pid.Kp, yaw2_angle_pid.Ki, yaw2_angle_pid.Kd, yaw2_angle_pid.KiPartDetachment);
//	    BasePID_Init(&attitude->Motors6020.turnPID[3], yaw2_speed_pid.Kp, yaw2_speed_pid.Ki, yaw2_speed_pid.Kd, yaw2_speed_pid.KiPartDetachment);
//    
//		attiTude.HWTdata.yall_angle_sum=0;
//}

//void Holder_Control(Attitude_t *attitude,RC_Ctrl *rc_ctrl)
//{
//	 if(rc_ctrl->key_SH_C_flag%2==0)   //返回值为6020返回值角度
//	 {
//		attitude->Motors6020.motor[1].Data.Output =BasePID_PitchSpeedControl((BasePID_Object*)(attitude->Motors6020.turnPID + 1) , BasePID_PitchAngleControl((BasePID_Object*)(attitude->Motors6020.turnPID + 0) , pitchangel, attiTude.Motors6020.motor[1].Data.Angle)  ,attiTude.Motors6020.motor[1].Data.SpeedRPM);
//	 }
//	 if(rc_ctrl->key_SH_C_flag%2==1)   //返回值使用陀螺仪yall轴累加
//	 {
//	
//		 attiTude.Pitch.Target_Angle=attiTude.Pitch.Target_Angle+ rc_ctrl->mouse.x*0.005;
//		
//		attitude->Motors6020.motor[1].Data.Output =BasePID_PitchSpeedControl((BasePID_Object*)(attitude->Motors6020.turnPID + 3) , BasePID_PitchAngleControl((BasePID_Object*)(attitude->Motors6020.turnPID + 2) , attiTude.Pitch.Target_Angle, attiTude.HWTdata.yall_angle_sum)  ,attiTude.HWTdata.yall_angle_speed);
//	  
//	 }
//	 
//		if(attitude->Motors6020.motor[1].Data.Output>=30000)  attitude->Motors6020.motor[1].Data.Output = 30000;
//   	if(attitude->Motors6020.motor[1].Data.Output<=-30000)  attitude->Motors6020.motor[1].Data.Output = -30000;

//		MotorFillData(&attitude->Motors6020.motor[1], attitude->Motors6020.motor[1].Data.Output);
//}

///**
//  * @brief  转塔控制（12900约等于一圈）
//  */
//void Attitude_Control(Attitude_t *attitude,RC_Ctrl *rc_ctrl)
//{
//		
//  	ctrl_motor1(&can1,0x02,attitude->Motors_DM.pos_yaw,1,30,0.1,0);

////		if(attitude->Motors2006.manual_rc_flag == 1)
////		{
////			if(rc_ctrl->rc.ch1-1024>300) 
////				attitude->Motors2006.target_angle += 10;
////			if(rc_ctrl->rc.ch1-1024<-300)
////				attitude->Motors2006.target_angle -= 10;
////		}

//			if(attitude->Motors2006.target_angle<=attitude->Motors2006.target_out)  //比较目标值和当前角度的大小，判断运动的方向
//			{
//				attitude->Motors2006.target_angle += 15;
//				if(attitude->Motors2006.target_angle>=attitude->Motors2006.target_out)
//					attitude->Motors2006.target_angle=attitude->Motors2006.target_out;
//			}
//			if(attitude->Motors2006.target_angle>attitude->Motors2006.target_out)  //比较目标值和当前角度的大小，判断运动的方向
//			{
//				attitude->Motors2006.target_angle -= 15;
//				if(attitude->Motors2006.target_angle<=attitude->Motors2006.target_out)
//					attitude->Motors2006.target_angle=attitude->Motors2006.target_out;
//			} 
//		
//	  //换算为累加角度
//		if((attitude->Motors2006.motor[0].Data.Angle<-100)&&(attitude->Motors2006.angle_last>100))
//		{
//		  attitude->Motors2006.angle +=360+attitude->Motors2006.motor[0].Data.Angle-attitude->Motors2006.angle_last;
//		}
//		else if((attitude->Motors2006.motor[0].Data.Angle>100)&&(attitude->Motors2006.angle_last<-100))
//		{
//		  attitude->Motors2006.angle +=-360+attitude->Motors2006.motor[0].Data.Angle-attitude->Motors2006.angle_last;
//		}
//		else 
//		{
//		  attitude->Motors2006.angle +=attitude->Motors2006.motor[0].Data.Angle-attitude->Motors2006.angle_last;
//		}
//		attitude->Motors2006.angle_last=attitude->Motors2006.motor[0].Data.Angle;
//		
//		attitude->Motors2006.turns_count = attitude->Motors2006.angle/12960;		//计算圈数
//		
//		//输出值PID解算
//		attitude->Motors2006.motor[0].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(attitude->Motors2006.RunPID+1) , BasePID_LiftingAngleControl\
//				((BasePID_Object*)(attitude->Motors2006.RunPID),attitude->Motors2006.target_angle, attitude->Motors2006.angle) ,attitude->Motors2006.motor[0].Data.SpeedRPM);
//		//写入数据
//		MotorFillData(&attitude->Motors2006.motor[0], attitude->Motors2006.motor[0].Data.Output);
//}	

///**
//  * @brief  转塔6020控制
//  */
//void Attitude_6020Control(Attitude_t *attitude,RC_Ctrl *rc_ctrl)
//{
////		if(rc_ctrl->isOnline == 1)		//遥控器离线检测
////	{
////		if(attitude->Motors6020.target_angle<=attitude->Motors6020.target_out)  //比较目标值和当前角度的大小，判断运动的方向
////		{
////			attitude->Motors6020.target_angle += pitch_sense;		//类似于缓启动
////				if(attitude->Motors6020.target_angle>=attitude->Motors6020.target_out)
////				{
////					attitude->Motors6020.target_angle = attitude->Motors6020.target_out;
////				}
////		}
////		if(attitude->Motors6020.target_angle>=attitude->Motors6020.target_out)  //比较目标值和当前角度的大小，判断运动的方向
////		{
////			attitude->Motors6020.target_angle -= pitch_sense;		//类似于缓启动
////				if(attitude->Motors6020.target_angle<=attitude->Motors6020.target_out)
////				{
////					attitude->Motors6020.target_angle = attitude->Motors6020.target_out;
////				}
////		}
////	}
////		if((rc_ctrl->rc.ch2-1024<-400)&&(rc_ctrl->isOnline==1)&&(rc_ctrl->rc.s2==3)	)
////		{
////			attitude->Motors3508.target_angle +=1;
////		}
////		else if((rc_ctrl->rc.ch2-1024>400)&&(rc_ctrl->isOnline==1)&&(rc_ctrl->rc.s2==3)	)
////		{
////			attitude->Motors3508.target_angle -=1;
////		}
////		
////		if(rc_ctrl->rc.s2==2)			//遥控器离线保护 
////		{
////			 attitude->Motors3508.target_angle = attitude->Motors3508.angle;
////		}
////		if(attitude->Motors6020.target_out <= -380)                        
////			attitude->Motors6020.target_out = -380;
////		else if(attitude->Motors6020.target_out >= 350)
////			attitude->Motors6020.target_out = 350;
//		
//		//角度累加换算
//		if((attitude->Motors6020.motor[1].Data.Angle<-100)&&(attitude->Motors6020.angle_last2>100))
//		{
//			attitude->Motors6020.angle2 += 360 + attitude->Motors6020.motor[1].Data.Angle - attitude->Motors6020.angle_last2;
//		}
//		else if(( attitude->Motors6020.motor[1].Data.Angle>100)&&(attitude->Motors6020.angle_last2<-100))
//		{
//			attitude->Motors6020.angle2 += -360 + attitude->Motors6020.motor[1].Data.Angle - attitude->Motors6020.angle_last2;
//		}
//		else 
//		{
//			attitude->Motors6020.angle2 += attitude->Motors6020.motor[1].Data.Angle - attitude->Motors6020.angle_last2;
//		}
//		attitude->Motors6020.angle_last2 = attitude->Motors6020.motor[1].Data.Angle;
//		
//		
//		
//		if((attitude->Motors6020.motor[0].Data.Angle<-100)&&(attitude->Motors6020.angle_last>100))
//		{
//			attitude->Motors6020.angle += 360 + attitude->Motors6020.motor[0].Data.Angle - attitude->Motors6020.angle_last;
//		}
//		else if(( attitude->Motors6020.motor[0].Data.Angle>100)&&(attitude->Motors6020.angle_last<-100))
//		{
//			attitude->Motors6020.angle += -360 + attitude->Motors6020.motor[0].Data.Angle - attitude->Motors6020.angle_last;
//		}
//		else 
//		{
//			attitude->Motors6020.angle += attitude->Motors6020.motor[0].Data.Angle - attitude->Motors6020.angle_last;
//		}
//		attitude->Motors6020.angle_last = attitude->Motors6020.motor[0].Data.Angle;
//		
//		attitude->Motors6020.motor[0].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(attitude->Motors6020.turnPID + 1) , BasePID_LiftingAngleControl((BasePID_Object*)(attitude->Motors6020.turnPID + 0) , attitude->Motors6020.target_angle, attitude->Motors6020.angle)  ,attitude->Motors6020.motor[0].Data.SpeedRPM);
//		attitude->Motors6020.motor[1].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(attitude->Motors6020.turnPID + 3) , BasePID_LiftingAngleControl((BasePID_Object*)(attitude->Motors6020.turnPID + 2) , attitude->Motors6020.target_angle2, attitude->Motors6020.angle2)  ,attitude->Motors6020.motor[1].Data.SpeedRPM);

//		
//		if(tim14.ClockTime%15==0)
//		{
//			attitude->Motors6020.target_angle = attitude->Motors6020.deg_last;
//			attitude->Motors6020.deg_last = attitude->Motors6020.angle;
//			attitude->Motors6020.target_angle2 = attitude->Motors6020.deg_last2;
//			attitude->Motors6020.deg_last2 = attitude->Motors6020.angle2;
//		}
//		if (attitude->Motors6020.motor[0].Data.Output >= 24000)
//			attitude->Motors6020.motor[0].Data.Output = 24000;
//		if (attitude->Motors6020.motor[0].Data.Output <= -24000)
//			attitude->Motors6020.motor[0].Data.Output = -24000;
//		
//		MotorFillData(&attitude->Motors6020.motor[0], attitude->Motors6020.motor[0].Data.Output);
//		MotorFillData(&attitude->Motors6020.motor[1], attitude->Motors6020.motor[1].Data.Output);
//}

///**
//  * @brief  转塔3508复位
//  */
//void Attitude_3508Reset(Attitude_t *attitude) 
//{
//	  error3508 = attitude->Motors3508.motor[0].Data.SpeedRPM-200;
//	  if(error3508>0)	error3508 = error3508;
//	  if(error3508<0)	error3508 = -error3508;

//		if((error3508<400)&&(tim14.ClockTime>100)&&(rscount2_3508>2000))	
//	{
//	   rscount1_3508++;
//		if(rscount1_3508>200)
//		{
//		attitude->Motors3508.motor[0].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(attitude->Motors3508.turnPID + 2),0,attitude->Motors3508.motor[0].Data.SpeedRPM);
//		  MotorFillData(&attitude->Motors3508.motor[0], attitude->Motors3508.motor[0].Data.Output);
//	    MotorCanOutput(can1, 0x1ff);
//		  MotorCanOutput(can1, 0x200);
//			rscount1_3508=200;
//		}
// }
//   else 
//	{
//	  attitude->Motors3508.motor[0].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(attitude->Motors3508.turnPID + 2),500,attitude->Motors3508.motor[0].Data.SpeedRPM);
//	if (attitude->Motors3508.motor[0].Data.Output >= 6000)
//		attitude->Motors3508.motor[0].Data.Output = 6000;
//	if (attitude->Motors3508.motor[0].Data.Output <= -6000)
//		attitude->Motors3508.motor[0].Data.Output = -6000;
//		MotorFillData(&attitude->Motors3508.motor[0], attitude->Motors3508.motor[0].Data.Output);
//	  MotorCanOutput(can1, 0x1ff);
//		MotorCanOutput(can1, 0x200);
//	  rscount2_3508++;
//		tim14.ClockTime=101;
//	}
//}


///**
//  * @brief  涵道电机控制
//  */
//void Bypass_motor_Control(RC_Ctrl *rc_ctrl)
//{

//	if((rc_ctrl->isOnline==1)&&(rc_ctrl->key_R_flag%2==0))
//	 {
//	     TIM3->CCR4=1100;
//	 }
//	  if((rc_ctrl->isOnline==1)&&((rc_ctrl->key_R_flag%2==1)||(bypass_motor_open_flag==1)))
//		{
//		  TIM3->CCR4=1900;   //TIM3->CCR4=1900;
//		}
//		if((rc_ctrl->isOnline==1)&&(rc_ctrl->key_RT_flag==1)&&(bypass_motor_open_flag==1))
//		{
//		  bypass_motor_open_flag=0;
//		  rc_ctrl->key_R_flag=0;
//		}
//		if(rc_ctrl->isOnline==0)
//		{
//		  TIM3->CCR4=1100;
//		}
//		if(rc_ctrl->key_SH_CT_B_flag%2==1&&rc_ctrl->key_R_flag%2!=1&&bypass_motor_open_flag==0)
//		{
//		   TIM3->CCR4=1350;
//		}
//		if(rc_ctrl->key_SH_CT_B_flag%2==0&&rc_ctrl->key_R_flag%2!=1&&bypass_motor_open_flag==0)
//		{
//		   TIM3->CCR4=1100;
//		}
//		if(rc_ctrl->key_R_flag%2==1||bypass_motor_open_flag==1)
//		{
//		   rc_ctrl->key_SH_CT_B_flag=0;
//		}
//}

uint8_t Gyro_callback(uint8_t * recBuffer, uint16_t len)
{
	angle_deal(recBuffer,&attiTude);
	return 0;
}

///**
//  * @brief   正点原子串口陀螺仪回调函数 ,波特率115200
//  * @param[in]  
//  */
void angle_deal(uint8_t *angle_receive,Attitude_t *attitude)
{
	if(angle_receive[0]==0x55 && angle_receive[1]==0x55)
	{
		if(angle_receive[15]==0x03)
		{
			attitude->HWTdata.pitch_angle_speed= -1*(float) ( (int16_t) (angle_receive[26]<<8) | angle_receive[25]) / 32768 * 2000;// * 0.01745;
			attitude->HWTdata.yall_angle_speed = -1*(float) ( (int16_t) (angle_receive[28]<<8) | angle_receive[27]) / 32768 * 2000;// * 0.01745;
		}
		if(angle_receive[2]==0x02)
		{
			attitude->HWTdata.Q0 = (float)((int16_t)(angle_receive[5]<<8 | angle_receive[4])) / 32768;
			attitude->HWTdata.Q1 = (float)((int16_t)(angle_receive[7]<<8 | angle_receive[6])) / 32768;
			attitude->HWTdata.Q2= (float)((int16_t)(angle_receive[9]<<8 | angle_receive[8])) / 32768;
			attitude->HWTdata.Q3= (float)((int16_t)(angle_receive[11]<<8 | angle_receive[10])) / 32768;
		}
		  
		  attiTude.HWTdata.yall_angle_last=attiTude.HWTdata.yall_angle;
  		attitude->HWTdata.yall_angle = -atan2(2 * attitude->HWTdata.Q1 * attitude->HWTdata.Q2 + 2 * attitude->HWTdata.Q0* attitude->HWTdata.Q3, -2 * attitude->HWTdata.Q2*attitude->HWTdata.Q2 - 2 * attitude->HWTdata.Q3* attitude->HWTdata.Q3 + 1)*RtA; 
      attitude->HWTdata.pitch_angle= -asin(-2 *attitude->HWTdata.Q1 * attitude->HWTdata.Q3 + 2 * attitude->HWTdata.Q0 * attitude->HWTdata.Q2)*RtA;
		  attitude->HWTdata.roll_angle = atan2(2 * attitude->HWTdata.Q2 * attitude->HWTdata.Q3 + 2 * attitude->HWTdata.Q0 * attitude->HWTdata.Q1, -2 * attitude->HWTdata.Q1 * attitude->HWTdata.Q1 - 2 * attitude->HWTdata.Q2* attitude->HWTdata.Q2 + 1)* RtA;  
      if(rc_Ctrl.key_SH_C_flag%2==1)  
			{		 
				if((attiTude.HWTdata.yall_angle<-100)&&(attiTude.HWTdata.yall_angle_last>100))
				{
					attiTude.HWTdata.yall_angle_sum +=360+attiTude.HWTdata.yall_angle-attiTude.HWTdata.yall_angle_last;
				}
				else if(( attiTude.HWTdata.yall_angle>100)&&(attiTude.HWTdata.yall_angle_last<-100))
				{
					 attiTude.HWTdata.yall_angle_sum +=-360+attiTude.HWTdata.yall_angle-attiTude.HWTdata.yall_angle_last;
				}
				else 
				{
					attiTude.HWTdata.yall_angle_sum +=attiTude.HWTdata.yall_angle-attiTude.HWTdata.yall_angle_last;
				}
//						  attiTude.HWTdata.yall_angle_last=attiTude.HWTdata.yall_angle;
				
//				Usart1DmaPrintf("%f,%f,%f,%f\r\n",attiTude.HWTdata.yall_angle,attiTude.HWTdata.yall_angle_last,attiTude.HWTdata.yall_angle_sum,		move_angle);

			}
			
	}
}