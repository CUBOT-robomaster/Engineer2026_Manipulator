//#include "lifting.h"
//#include "hardware_config.h"
//#include<dr16.h>
//#include<driver_timer.h>
//#include <ore.h>
//#define abs(x) ((x)>0? (x):(-(x)))
//#define lifting_firstpart 0
//#define lifting_secondpart 1

//float left_up_limit = -10000;
//float right_up_limt = -6000;
//float left_down_limit = 6000;
//float right_down_limt = 10000;
//float limit2 = 10350;
//float lifting_vel_param = 8;  
//float lifting_ore_vel_param = 25;		//直接位置控制-抬升速度参数,8
//float lifting_ore2_vel_param = 10.5;	

//float lifting_first_param = 1;
//float lifting_second_param = 1;
//float leftsince=1;
//float rightsince=1;
//int16_t error1;
//int16_t error2;

//int16_t error3;
//int16_t error4;

//int16_t error5;
//int16_t error6;
//uint16_t resetlimit = 5;
//uint16_t resetcount=0;
//uint16_t resetcount1=0;
//uint16_t Motoroutputlimit=10000;
//uint16_t Motoroutputlimit_2=8000;

///*以下为抬升限幅*/
//float lifting_limit_min=0;
//float lifting_limit_1st_max=29344;

//float lift2_limit_min = -10000;
//float lift2_limit_max = 0;

//float lifting_limit_2nd_min = 30000;  //-11858  //17400
//float lifting_limit_2nd_max = -30000;

//uint8_t plugging_reset_flag=0;
//uint8_t plugging_reset_flag1=0;
//uint8_t plugging_reset_flag_one=0;
//float plugging_reset_time=0;
//float plugging_reset_time1=0;
//uint16_t resetcount2=0;
//uint16_t resetcount3=0;
///**
//*  @breif 抬升机构电机初始化
//*  @note  初始化两段抬升的四个电机
//*/
//void Lifting_first_Init(Lifting_t *lifting,BasePID_Object pid_left_1_angle,BasePID_Object pid_left_1_speed,BasePID_Object pid_right_1_angle,BasePID_Object pid_right_1_speed,CanNumber canx)
//{
//	 lifting->Auto_State = 0;
//   MotorInit(&lifting->Motors3508.motor[0],  4174, Motor3508, canx , 0x205);		//一段抬升
//	 MotorInit(&lifting->Motors3508.motor[1],  3534, Motor3508, canx , 0x206);
//	
//   //一段抬升PID初始化
//	 BasePID_Init(&lifting->Motors3508.turnPID[0], pid_left_1_angle.Kp, pid_left_1_angle.Ki, pid_left_1_angle.Kd, pid_left_1_angle.KiPartDetachment);
//	 BasePID_Init(&lifting->Motors3508.turnPID[1], pid_left_1_speed.Kp, pid_left_1_speed.Ki, pid_left_1_speed.Kd, pid_left_1_speed.KiPartDetachment);
//	 BasePID_Init(&lifting->Motors3508.turnPID[2], pid_right_1_angle.Kp, pid_right_1_angle.Ki, pid_right_1_angle.Kd, pid_right_1_angle.KiPartDetachment);
//	 BasePID_Init(&lifting->Motors3508.turnPID[3], pid_right_1_speed.Kp, pid_right_1_speed.Ki, pid_right_1_speed.Kd, pid_right_1_speed.KiPartDetachment);
//}

//void Lifting_second_Init(Lifting_t *lifting,BasePID_Object pid_left_2_angle,BasePID_Object pid_left_2_speed,BasePID_Object pid_right_2_angle,BasePID_Object pid_right_2_speed,BasePID_Object reset_left,BasePID_Object reset_right,CanNumber canx)
//{
//   MotorInit(&lifting->Motors3508.motor[2],  4174, Motor3508, CAN1 , 0x207);		//二段抬升
//	 MotorInit(&lifting->Motors3508.motor[3],  3534, Motor3508, CAN1 , 0x208);

//	 //二段抬升PID初始化
//	 BasePID_Init(&lifting->Motors3508.turnPID[4], pid_left_2_angle.Kp, pid_left_2_angle.Ki, pid_left_2_angle.Kd, pid_left_2_angle.KiPartDetachment);
//	 BasePID_Init(&lifting->Motors3508.turnPID[5], pid_left_2_speed.Kp, pid_left_2_speed.Ki, pid_left_2_speed.Kd, pid_left_2_speed.KiPartDetachment);
//	 BasePID_Init(&lifting->Motors3508.turnPID[6], pid_right_2_angle.Kp, pid_right_2_angle.Ki, pid_right_2_angle.Kd, pid_right_2_angle.KiPartDetachment);
//	 BasePID_Init(&lifting->Motors3508.turnPID[7], pid_right_2_speed.Kp, pid_right_2_speed.Ki, pid_right_2_speed.Kd, pid_right_2_speed.KiPartDetachment);
//	 BasePID_Init(&lifting->Motors3508.turnPID[8], reset_left.Kp, reset_left.Ki, reset_left.Kd, reset_left.KiPartDetachment);
//	 BasePID_Init(&lifting->Motors3508.turnPID[9], reset_right.Kp, reset_right.Ki, reset_right.Kd, reset_right.KiPartDetachment);

//}


//void Lifting_Reset(Lifting_t *lifting) 
//{
//	  error1=lifting->Motors3508.motor[2].Data.SpeedRPM-200;
//	  error2=lifting->Motors3508.motor[3].Data.SpeedRPM-200;
//	if(error1>0)error1=error1;
//	if(error1<0)error1=-error1;
//	if(error2>0)error2=error2;
//	if(error2<0)error2=-error2;
//if((error1<1100)&&(error2<800)&&(tim14.ClockTime>100)&&(resetcount1>2000))	
//	{
//	   resetcount++;
//		if(resetcount>80)
//		{
//		lifting->Motors3508.motor[2].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 8),0.2*resetlimit,lifting->Motors3508.motor[2].Data.SpeedRPM);
//		lifting->Motors3508.motor[3].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 9),-0.4*resetlimit ,lifting->Motors3508.motor[3].Data.SpeedRPM);
//		  MotorFillData(&lifting->Motors3508.motor[2], lifting->Motors3508.motor[2].Data.Output);
//     	MotorFillData(&lifting->Motors3508.motor[3], lifting->Motors3508.motor[3].Data.Output);
//	    MotorCanOutput(can1, 0x1ff);
//		  MotorCanOutput(can1, 0x200);
//			resetcount=200;
//			lifting->Lifting_left_2.angle=0;
//			lifting->Lifting_right_2.angle=0;
//		}
// }
//  else 
//	{
//	  lifting->Motors3508.motor[2].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 8),+3*resetlimit,lifting->Motors3508.motor[2].Data.SpeedRPM);
//		lifting->Motors3508.motor[3].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 9),-3*resetlimit,lifting->Motors3508.motor[3].Data.SpeedRPM);
//	if (lifting->Motors3508.motor[2].Data.Output >= 6000)
//		lifting->Motors3508.motor[2].Data.Output = 6000;
//	if (lifting->Motors3508.motor[2].Data.Output <= -6000)
//		lifting->Motors3508.motor[2].Data.Output = -6000;
//	if (lifting->Motors3508.motor[3].Data.Output >= 6000)
//		lifting->Motors3508.motor[3].Data.Output = 6000;
//	if (lifting->Motors3508.motor[3].Data.Output <= -6000)
//		lifting->Motors3508.motor[3].Data.Output = -6000;
//		MotorFillData(&lifting->Motors3508.motor[2], lifting->Motors3508.motor[2].Data.Output);
//   	MotorFillData(&lifting->Motors3508.motor[3], lifting->Motors3508.motor[3].Data.Output);
//	  MotorCanOutput(can1, 0x1ff);
//		MotorCanOutput(can1, 0x200);
//	  resetcount1++;
//		tim14.ClockTime=101;
//	}
//}




////void plugging_reset(RC_Ctrl *rc_ctrl,Lifting_t *lifting)
////{
////   if(plugging_reset_flag==1)
////	 {
////	     plugging_reset_time++;
////				 
////			lifting->Motors3508.motor[2].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 8),+resetlimit*4,lifting->Motors3508.motor[2].Data.SpeedRPM);
////			lifting->Motors3508.motor[3].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 9),-resetlimit*5,lifting->Motors3508.motor[3].Data.SpeedRPM);
////			if (lifting->Motors3508.motor[2].Data.Output >= 4500)
////				lifting->Motors3508.motor[2].Data.Output = 4500;
////			if (lifting->Motors3508.motor[2].Data.Output <= -4500)
////				lifting->Motors3508.motor[2].Data.Output = -4500;
////			if (lifting->Motors3508.motor[3].Data.Output >= 5500)
////				lifting->Motors3508.motor[3].Data.Output = 5500;
////			if (lifting->Motors3508.motor[3].Data.Output <= -5500)
////				lifting->Motors3508.motor[3].Data.Output = -5500;
////			MotorFillData(&lifting->Motors3508.motor[2], lifting->Motors3508.motor[2].Data.Output);
////			MotorFillData(&lifting->Motors3508.motor[3], lifting->Motors3508.motor[3].Data.Output);
////			
////			  error3=lifting->Motors3508.motor[2].Data.SpeedRPM-200;
////				error4=lifting->Motors3508.motor[3].Data.SpeedRPM-200;
////			if(error3>0)error3=error3;
////			if(error3<0)error3=-error3;
////			if(error4>0)error4=error4;
////			if(error4<0)error4=-error4;
////			if(plugging_reset_time>800)
////			{
////			if((error3<600)&&(error4<600))	
////				{
////					 resetcount2++;
////					if(resetcount2>80)
////					{
////					lifting->Motors3508.motor[2].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 8),0,lifting->Motors3508.motor[2].Data.SpeedRPM);
////					lifting->Motors3508.motor[3].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 9),0 ,lifting->Motors3508.motor[3].Data.SpeedRPM);
////					MotorFillData(&lifting->Motors3508.motor[2], lifting->Motors3508.motor[2].Data.Output);
////					MotorFillData(&lifting->Motors3508.motor[3], lifting->Motors3508.motor[3].Data.Output);
////						resetcount2=0;
////						lifting->Lifting_left_2.Target_Angle = 0;
////						lifting->Lifting_right_2.Target_Angle = 0;
////						lifting->Lifting_left_2.angle = 0;
////						lifting->Lifting_right_2.angle = 0;
////						plugging_reset_flag=0;
////						plugging_reset_time=0;
////					}
////			  }
////			}
////		 
////   }
////}
//void plugging_reset(RC_Ctrl *rc_ctrl,Lifting_t *lifting)//二段抬升限位开关复位
//{
//	if(plugging_reset_flag==1&&(rc_ctrl->isOnline==1))
//	{
//		lifting->Lifting_left_2.Target_Angle+=resetlimit;
//		lifting->Lifting_right_2.Target_Angle-=resetlimit;
//		//转换为累加角度
//	  if((lifting->Motors3508.motor[2].Data.Angle<-80)&&(lifting->Lifting_left_2.angle_last>80))
//		{
//		  lifting->Lifting_left_2.angle +=360+lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[2].Data.Angle>80)&&(lifting->Lifting_left_2.angle_last<-80))
//		{
//		  lifting->Lifting_left_2.angle +=-360+lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_left_2.angle +=lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		 lifting->Lifting_left_2.angle_last=lifting->Motors3508.motor[2].Data.Angle;

//    
//		if((lifting->Motors3508.motor[3].Data.Angle<-80)&&(lifting->Lifting_right_2.angle_last>80))
//		{
//		  lifting->Lifting_right_2.angle +=360+lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[3].Data.Angle>80)&&(lifting->Lifting_right_2.angle_last<-80))
//		{
//		  lifting->Lifting_right_2.angle +=-360+lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_right_2.angle +=lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		 lifting->Lifting_right_2.angle_last=lifting->Motors3508.motor[3].Data.Angle;
//		
//		lifting->Motors3508.motor[2].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 5) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 4) , lifting->Lifting_left_2.Target_Angle, lifting->Lifting_left_2.angle)  ,lifting->Motors3508.motor[2].Data.SpeedRPM);
//		lifting->Motors3508.motor[3].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 7) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 6) , lifting->Lifting_right_2.Target_Angle, lifting->Lifting_right_2.angle)  , lifting->Motors3508.motor[3].Data.SpeedRPM);
// 
//    if(lifting->Motors3508.motor[2].Data.Output>=Motoroutputlimit)  lifting->Motors3508.motor[2].Data.Output = Motoroutputlimit;
//   	if(lifting->Motors3508.motor[2].Data.Output<=-Motoroutputlimit)  lifting->Motors3508.motor[2].Data.Output = -Motoroutputlimit;
//		if(lifting->Motors3508.motor[3].Data.Output>=Motoroutputlimit)  lifting->Motors3508.motor[3].Data.Output = Motoroutputlimit;
//   	if(lifting->Motors3508.motor[3].Data.Output<=-Motoroutputlimit)  lifting->Motors3508.motor[3].Data.Output = -Motoroutputlimit;

//		//写入电机数据
//		MotorFillData(&lifting->Motors3508.motor[2], lifting->Motors3508.motor[2].Data.Output);
//   	MotorFillData(&lifting->Motors3508.motor[3], lifting->Motors3508.motor[3].Data.Output);

//		if (lifting->Motors3508.motor[2].Data.Output >= 10000)
//		lifting->Motors3508.motor[2].Data.Output = 10000;
//			if (lifting->Motors3508.motor[2].Data.Output <= -10000)
//				lifting->Motors3508.motor[2].Data.Output = -10000;
//			if (lifting->Motors3508.motor[3].Data.Output >= 10000)
//				lifting->Motors3508.motor[3].Data.Output = 10000;
//			if (lifting->Motors3508.motor[3].Data.Output <= -10000)
//				lifting->Motors3508.motor[3].Data.Output = -10000;
//	  

//			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)==1)
//			{


//						lifting->Lifting_left_2.Target_Angle = 0;
//						lifting->Lifting_right_2.Target_Angle = 0;
//						lifting->Lifting_left_2.angle = 0;
//						lifting->Lifting_right_2.angle = 0;
//						plugging_reset_flag=0;
//			}
//	
//	}

//}
//void plugging_reset1(RC_Ctrl *rc_ctrl,Lifting_t *lifting)
//{
//   if(plugging_reset_flag1==1)
//	 {
//	     plugging_reset_time1++;
//		 if(plugging_reset_time1==1)
//		 {
//		   Auto_state = 1; 
//		 }
//		 if(plugging_reset_time1<1500)
//		 {
//		   liftIng.lifting_target_1 = 2324;   
//		 }
//     if(plugging_reset_time1==1500)
//		 {
//			 Auto_state = 0; 
//		 }
//		 if(plugging_reset_time1>1500)
//		 {
//			  lifting->Motors3508.motor[0].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 8),1.5*resetlimit,lifting->Motors3508.motor[0].Data.SpeedRPM);
//				lifting->Motors3508.motor[1].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 9),-resetlimit*1.5,lifting->Motors3508.motor[1].Data.SpeedRPM);
//			if (lifting->Motors3508.motor[0].Data.Output >= 3000)
//				lifting->Motors3508.motor[0].Data.Output = 3000;
//			if (lifting->Motors3508.motor[0].Data.Output <= -3000)
//				lifting->Motors3508.motor[0].Data.Output = -3000;
//			if (lifting->Motors3508.motor[1].Data.Output >= 3000)
//				lifting->Motors3508.motor[1].Data.Output = 3000;
//			if (lifting->Motors3508.motor[1].Data.Output <= -3000)
//				lifting->Motors3508.motor[1].Data.Output = -3000;
//				MotorFillData(&lifting->Motors3508.motor[0], lifting->Motors3508.motor[0].Data.Output);
//				MotorFillData(&lifting->Motors3508.motor[1], lifting->Motors3508.motor[1].Data.Output);
//			
//			  error5=lifting->Motors3508.motor[0].Data.SpeedRPM-200;
//				error6=lifting->Motors3508.motor[1].Data.SpeedRPM-200;
//			
//			if(error5>0)error5=error5;
//			if(error5<0)error5=-error5;
//			if(error6>0)error6=error6;
//			if(error6<0)error6=-error6;
//			if(plugging_reset_time1>2500)
//			{
//			if((error3<600)&&(error4<600))	
//				{
//					 resetcount3++;
//					if(resetcount3>200)
//					{
//					lifting->Motors3508.motor[0].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 8),0,lifting->Motors3508.motor[0].Data.SpeedRPM);
//					lifting->Motors3508.motor[1].Data.Output=BasePID_SidewaySpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 9),0 ,lifting->Motors3508.motor[1].Data.SpeedRPM);
//						MotorFillData(&lifting->Motors3508.motor[0], lifting->Motors3508.motor[0].Data.Output);
//						MotorFillData(&lifting->Motors3508.motor[1], lifting->Motors3508.motor[1].Data.Output);
//						plugging_reset_time1=0;
//						resetcount3 = 0;
//						lifting->Lifting_left_1.angle=0;
//						lifting->Lifting_right_1.angle=0;
//						lifting->Lifting_left_1.Target_Angle=0;
//			      lifting->Lifting_right_1.Target_Angle=0;
//						plugging_reset_flag1=0;
//					}
//			  }
//			}
//		 }
//   }
//}

///**
//*  @breif 抬升机构遥控控制
//*  @note  用于测试
//*/
//void LiftingGetRemoteData(Lifting_t *lifting,RC_Ctrl *rc_ctrl)
//{
//			//一段抬升
//		if(Controller_flag==0)
//		{
//	    if(rc_ctrl->rc.s1==1&&(rc_ctrl->isOnline==1))
//			{
//			    lifting->Lifting_left_1.Target_Angle-=lifting_vel_param;
//			    lifting->Lifting_right_1.Target_Angle+=lifting_vel_param;
//			}
//			else if(rc_ctrl->rc.s1==2&&(rc_ctrl->isOnline==1))
//			{
//			    lifting->Lifting_left_1.Target_Angle+=lifting_vel_param;
//			    lifting->Lifting_right_1.Target_Angle-=lifting_vel_param;
//			
//			}
//		}
//			else if(Controller_flag==1&&Custom.image_recv.Coordinate.z>-150)
//			{
//			if((Custom.image_recv.Coordinate.z)>50&&(rc_ctrl->isOnline==1))
//			{
//			    lifting->Lifting_left_1.Target_Angle-=lifting_vel_param;
//			    lifting->Lifting_right_1.Target_Angle+=lifting_vel_param;
//			}
//			else if((Custom.image_recv.Coordinate.z)<-50&&(rc_ctrl->isOnline==1))
//			{
//			    lifting->Lifting_left_1.Target_Angle+=lifting_vel_param;
//			    lifting->Lifting_right_1.Target_Angle-=lifting_vel_param;
//			
//			}
//		}
//			if(rc_ctrl->isOnline==0)			//遥控器离线保护 
//			{
//			   lifting->Lifting_left_1.Target_Angle=lifting->Lifting_left_1.angle;
//			   lifting->Lifting_right_1.Target_Angle=lifting->Lifting_right_1.angle;
//			}

//   	//	运动范围限幅
//    if(lifting->Lifting_left_1.Target_Angle <= -lifting_limit_1st_max)                        
//    lifting->Lifting_left_1.Target_Angle = -lifting_limit_1st_max;
//    else if(lifting->Lifting_left_1.Target_Angle >= lifting_limit_min)
//    lifting->Lifting_left_1.Target_Angle = lifting_limit_min;
//		
//		if(lifting->Lifting_right_1.Target_Angle <= lifting_limit_min)                        
//    lifting->Lifting_right_1.Target_Angle = lifting_limit_min;
//    else if(lifting->Lifting_right_1.Target_Angle >= lifting_limit_1st_max)
//    lifting->Lifting_right_1.Target_Angle = lifting_limit_1st_max;
//		
//		//转换为累加角度
//	  if((lifting->Motors3508.motor[0].Data.Angle<-100)&&(lifting->Lifting_left_1.angle_last>100))
//		{
//		  lifting->Lifting_left_1.angle +=360+lifting->Motors3508.motor[0].Data.Angle-lifting->Lifting_left_1.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[0].Data.Angle>100)&&(lifting->Lifting_left_1.angle_last<-100))
//		{
//		  lifting->Lifting_left_1.angle +=-360+lifting->Motors3508.motor[0].Data.Angle-lifting->Lifting_left_1.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_left_1.angle +=lifting->Motors3508.motor[0].Data.Angle-lifting->Lifting_left_1.angle_last;
//		}
//		 lifting->Lifting_left_1.angle_last=lifting->Motors3508.motor[0].Data.Angle;

//    
//		if((lifting->Motors3508.motor[1].Data.Angle<-100)&&(lifting->Lifting_right_1.angle_last>100))
//		{
//		  lifting->Lifting_right_1.angle +=360+lifting->Motors3508.motor[1].Data.Angle-lifting->Lifting_right_1.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[1].Data.Angle>100)&&(lifting->Lifting_right_1.angle_last<-100))
//		{
//		  lifting->Lifting_right_1.angle +=-360+lifting->Motors3508.motor[1].Data.Angle-lifting->Lifting_right_1.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_right_1.angle +=lifting->Motors3508.motor[1].Data.Angle-lifting->Lifting_right_1.angle_last;
//		}
//		 lifting->Lifting_right_1.angle_last=lifting->Motors3508.motor[1].Data.Angle;
//		
//		lifting->Motors3508.motor[0].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 1) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 0) , lifting->Lifting_left_1.Target_Angle, lifting->Lifting_left_1.angle)  ,lifting->Motors3508.motor[0].Data.SpeedRPM);
//		lifting->Motors3508.motor[1].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 3) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 2) , lifting->Lifting_right_1.Target_Angle, lifting->Lifting_right_1.angle)  , lifting->Motors3508.motor[1].Data.SpeedRPM);
// 
//    if(lifting->Motors3508.motor[0].Data.Output>=Motoroutputlimit)  lifting->Motors3508.motor[0].Data.Output = Motoroutputlimit;
//   	if(lifting->Motors3508.motor[0].Data.Output<=-Motoroutputlimit)  lifting->Motors3508.motor[0].Data.Output = -Motoroutputlimit;
//		if(lifting->Motors3508.motor[1].Data.Output>=Motoroutputlimit)  lifting->Motors3508.motor[1].Data.Output = Motoroutputlimit;
//   	if(lifting->Motors3508.motor[1].Data.Output<=-Motoroutputlimit)  lifting->Motors3508.motor[1].Data.Output = -Motoroutputlimit;

//		//写入电机数据
//		MotorFillData(&lifting->Motors3508.motor[1], lifting->Motors3508.motor[1].Data.Output);
//   	MotorFillData(&lifting->Motors3508.motor[0], lifting->Motors3508.motor[0].Data.Output);

//		if(plugging_reset_flag!=1)
//		{
//		//二段抬升
//		if((rc_ctrl->rc.ch3-1024>400)&&(rc_ctrl->isOnline==1))      //if(rc_ctrl->rc.ch1-1024<-300)
//		{
//				lifting->Lifting_left_2.Target_Angle+=lifting_vel_param;
//				lifting->Lifting_right_2.Target_Angle-=lifting_vel_param;
//		}
//		else if((rc_ctrl->rc.ch3-1024<-400)&&(rc_ctrl->isOnline==1))
//		{
//				lifting->Lifting_left_2.Target_Angle-=lifting_vel_param;
//				lifting->Lifting_right_2.Target_Angle+=lifting_vel_param;
//		}
//		if(rc_ctrl->isOnline==0)			//遥控器离线保护 
//			
//		{
//			 lifting->Lifting_left_2.Target_Angle=lifting->Lifting_left_2.angle;
//			 lifting->Lifting_right_2.Target_Angle=lifting->Lifting_right_2.angle;
//		}

////		float lift2_limit_min = -10000;
////float lift2_limit_max = 0;
//   	//	运动范围限幅
//		//limit2 = 8862 + lifting->Lifting_right_1.Target_Angle;
//		if(manual_mining_flag==0)
//		{
//			if(lifting->Lifting_left_2.Target_Angle <= -limit2) 
//				lifting->Lifting_left_2.Target_Angle = -limit2;
//			else if(lifting->Lifting_left_2.Target_Angle >= 0)
//				lifting->Lifting_left_2.Target_Angle = 0;
//			
//			if(lifting->Lifting_right_2.Target_Angle <= 0)                        
//				lifting->Lifting_right_2.Target_Angle = 0;
//			else if(lifting->Lifting_right_2.Target_Angle >= limit2)
//				lifting->Lifting_right_2.Target_Angle = limit2;
//		}
//		else if((manual_mining_flag==1)||(conver_reset1==1))	//进入手动兑矿模式，释放二级抬升限幅
//		{
//			if(lifting->Lifting_left_2.Target_Angle <= -20000) 
//				lifting->Lifting_left_2.Target_Angle = -20000;
//			else if(lifting->Lifting_left_2.Target_Angle >= 0)
//				lifting->Lifting_left_2.Target_Angle = 0;
//			
//			if(lifting->Lifting_right_2.Target_Angle <= 0)                        
//				lifting->Lifting_right_2.Target_Angle = 0;
//			else if(lifting->Lifting_right_2.Target_Angle >= 20000)
//				lifting->Lifting_right_2.Target_Angle = 20000;			
//		}
//		//转换为累加角度
//	  if((lifting->Motors3508.motor[2].Data.Angle<-80)&&(lifting->Lifting_left_2.angle_last>80))
//		{
//		  lifting->Lifting_left_2.angle +=360+lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[2].Data.Angle>80)&&(lifting->Lifting_left_2.angle_last<-80))
//		{
//		  lifting->Lifting_left_2.angle +=-360+lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_left_2.angle +=lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		 lifting->Lifting_left_2.angle_last=lifting->Motors3508.motor[2].Data.Angle;

//    
//		if((lifting->Motors3508.motor[3].Data.Angle<-80)&&(lifting->Lifting_right_2.angle_last>80))
//		{
//		  lifting->Lifting_right_2.angle +=360+lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[3].Data.Angle>80)&&(lifting->Lifting_right_2.angle_last<-80))
//		{
//		  lifting->Lifting_right_2.angle +=-360+lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_right_2.angle +=lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		 lifting->Lifting_right_2.angle_last=lifting->Motors3508.motor[3].Data.Angle;
//		
//		lifting->Motors3508.motor[2].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 5) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 4) , lifting->Lifting_left_2.Target_Angle, lifting->Lifting_left_2.angle)  ,lifting->Motors3508.motor[2].Data.SpeedRPM);
//		lifting->Motors3508.motor[3].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 7) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 6) , lifting->Lifting_right_2.Target_Angle, lifting->Lifting_right_2.angle)  , lifting->Motors3508.motor[3].Data.SpeedRPM);
// 
//    if(lifting->Motors3508.motor[2].Data.Output>=Motoroutputlimit)  lifting->Motors3508.motor[2].Data.Output = Motoroutputlimit;
//   	if(lifting->Motors3508.motor[2].Data.Output<=-Motoroutputlimit)  lifting->Motors3508.motor[2].Data.Output = -Motoroutputlimit;
//		if(lifting->Motors3508.motor[3].Data.Output>=Motoroutputlimit)  lifting->Motors3508.motor[3].Data.Output = Motoroutputlimit;
//   	if(lifting->Motors3508.motor[3].Data.Output<=-Motoroutputlimit)  lifting->Motors3508.motor[3].Data.Output = -Motoroutputlimit;

//		//写入电机数据
//		MotorFillData(&lifting->Motors3508.motor[2], lifting->Motors3508.motor[2].Data.Output);
//   	MotorFillData(&lifting->Motors3508.motor[3], lifting->Motors3508.motor[3].Data.Output);
//	}
//}

///**
//*  @breif 抬升直接位置控制函数 
//*  @note  用于在自动流程中控制抬升运动到某一高度,tongguo
//*/
//void Lifting_Ore_firstpart(Lifting_t *lifting,RC_Ctrl *rc_ctrl,float lifting_move_distance)
//{
////		if(auto_sliver_slow_flag==1)
////			lifting_ore_vel_param=26*0.8;
////		if(auto_sliver_slow_flag==0)
////			lifting_ore_vel_param=26;
//	float target_1;
//	target_1=lifting_move_distance*lifting_first_param;			//换算公式
//	int direction_flag_1=0;

//	if(rc_ctrl->isOnline == 1)			//遥控器离线检测
//	{
//		if(lifting->Lifting_right_1.Target_Angle<=target_1)  //比较目标值和当前角度的大小，判断运动的方向
//		{
//					lifting->Lifting_left_1.Target_Angle -= lifting_ore_vel_param;		//类似于缓启动
//					lifting->Lifting_right_1.Target_Angle += lifting_ore_vel_param;
//			    if(lifting->Lifting_right_1.Target_Angle>=target_1)
//					{
//					   lifting->Lifting_right_1.Target_Angle=target_1;
//						 lifting->Lifting_left_1.Target_Angle=-target_1;
//					}

//		}
//		if(lifting->Lifting_right_1.Target_Angle>target_1)  //比较目标值和当前角度的大小，判断运动的方向
//		{
//					lifting->Lifting_left_1.Target_Angle += lifting_ore_vel_param;		//类似于缓启动
//					lifting->Lifting_right_1.Target_Angle -= lifting_ore_vel_param;
//			  if(lifting->Lifting_right_1.Target_Angle<=target_1)
//					{
//					   lifting->Lifting_right_1.Target_Angle=target_1;
//						 lifting->Lifting_left_1.Target_Angle=-target_1;
//					}
//		}		
//	}	 
//	if(rc_ctrl->isOnline==0)		//若遥控器离线，运动停止
//	{
//		lifting->Lifting_left_1.Target_Angle=lifting->Lifting_left_1.angle;
//		lifting->Lifting_right_1.Target_Angle=lifting->Lifting_right_1.angle;
//	}
//				
//		
//		 
////	运动范围限幅
//		if(lifting->Lifting_left_1.Target_Angle <= -lifting_limit_1st_max)                        
//		lifting->Lifting_left_1.Target_Angle = -lifting_limit_1st_max;
//		else if(lifting->Lifting_left_1.Target_Angle >= lifting_limit_min)
//		lifting->Lifting_left_1.Target_Angle = lifting_limit_min;
//		
//		if(lifting->Lifting_right_1.Target_Angle <= lifting_limit_min)                        
//		lifting->Lifting_right_1.Target_Angle = lifting_limit_min;
//		else if(lifting->Lifting_right_1.Target_Angle >= lifting_limit_1st_max)
//		lifting->Lifting_right_1.Target_Angle = lifting_limit_1st_max;
//			
//		if((lifting->Motors3508.motor[0].Data.Angle<-80)&&(lifting->Lifting_left_1.angle_last>80))
//		{
//			lifting->Lifting_left_1.angle +=360+lifting->Motors3508.motor[0].Data.Angle-lifting->Lifting_left_1.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[0].Data.Angle>80)&&(lifting->Lifting_left_1.angle_last<-80))
//		{
//			lifting->Lifting_left_1.angle +=-360+lifting->Motors3508.motor[0].Data.Angle-lifting->Lifting_left_1.angle_last;
//		}
//		else 
//		{
//			lifting->Lifting_left_1.angle +=lifting->Motors3508.motor[0].Data.Angle-lifting->Lifting_left_1.angle_last;
//		}
//		 lifting->Lifting_left_1.angle_last=lifting->Motors3508.motor[0].Data.Angle;

//		
//		if((lifting->Motors3508.motor[1].Data.Angle<-80)&&(lifting->Lifting_right_1.angle_last>80))
//		{
//			lifting->Lifting_right_1.angle +=360+lifting->Motors3508.motor[1].Data.Angle-lifting->Lifting_right_1.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[1].Data.Angle>80)&&(lifting->Lifting_right_1.angle_last<-80))
//		{
//			lifting->Lifting_right_1.angle +=-360+lifting->Motors3508.motor[1].Data.Angle-lifting->Lifting_right_1.angle_last;
//		}
//		else 
//		{
//			lifting->Lifting_right_1.angle +=lifting->Motors3508.motor[1].Data.Angle-lifting->Lifting_right_1.angle_last;
//		}
//		 lifting->Lifting_right_1.angle_last=lifting->Motors3508.motor[1].Data.Angle;
//		
//		lifting->Motors3508.motor[0].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 1) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 0) , lifting->Lifting_left_1.Target_Angle, lifting->Lifting_left_1.angle)  ,lifting->Motors3508.motor[0].Data.SpeedRPM);
//		lifting->Motors3508.motor[1].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 3) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 2) , lifting->Lifting_right_1.Target_Angle , lifting->Lifting_right_1.angle)  , lifting->Motors3508.motor[1].Data.SpeedRPM);
// 
//		if(lifting->Motors3508.motor[0].Data.Output>=Motoroutputlimit)  lifting->Motors3508.motor[0].Data.Output = Motoroutputlimit;
//		if(lifting->Motors3508.motor[0].Data.Output<=-Motoroutputlimit)  lifting->Motors3508.motor[0].Data.Output = -Motoroutputlimit;
//		if(lifting->Motors3508.motor[1].Data.Output>=Motoroutputlimit)  lifting->Motors3508.motor[1].Data.Output = Motoroutputlimit;
//		if(lifting->Motors3508.motor[1].Data.Output<=-Motoroutputlimit)  lifting->Motors3508.motor[1].Data.Output = -Motoroutputlimit;

//		MotorFillData(&lifting->Motors3508.motor[1], lifting->Motors3508.motor[1].Data.Output);
//		MotorFillData(&lifting->Motors3508.motor[0], lifting->Motors3508.motor[0].Data.Output);	

//}

//void Lifting_Ore_secondpart(Lifting_t *lifting,RC_Ctrl *rc_ctrl,float lifting_move_distance)
//{
//		float target_2;
//		target_2=lifting_move_distance*lifting_second_param;			//换算公式
//		int direction_flag_2=0;
//	
//		if(rc_ctrl->isOnline == 1)			//遥控器离线检测
//		{
//		
//					if(lifting->Lifting_right_2.Target_Angle<=target_2)
//					{
//						lifting->Lifting_left_2.Target_Angle -= lifting_ore2_vel_param;		//类似于缓启动
//						lifting->Lifting_right_2.Target_Angle += lifting_ore2_vel_param;
//						if(lifting->Lifting_right_2.Target_Angle>=target_2)
//						{
//						  lifting->Lifting_right_2.Target_Angle=target_2;
//							lifting->Lifting_left_2.Target_Angle = -target_2;
//						}
//					}
//				
//					if(lifting->Lifting_right_2.Target_Angle>target_2)
//					{
//						lifting->Lifting_left_2.Target_Angle += lifting_ore2_vel_param;
//						lifting->Lifting_right_2.Target_Angle -= lifting_ore2_vel_param;
//						if(lifting->Lifting_right_2.Target_Angle<=target_2)
//						{
//						  lifting->Lifting_right_2.Target_Angle=target_2;
//							lifting->Lifting_left_2.Target_Angle = -target_2;
//						}
//					}
//		}	 
//		if(rc_ctrl->isOnline==0)		//若遥控器离线，运动停止
//		{
//			lifting->Lifting_left_2.Target_Angle=lifting->Lifting_left_2.angle;
//			lifting->Lifting_right_2.Target_Angle=lifting->Lifting_right_2.angle;
//		}
//		//	运动范围限幅
//		if(lifting->Lifting_left_2.Target_Angle <= -20000) 
//			lifting->Lifting_left_2.Target_Angle = -20000;
//    else if(lifting->Lifting_left_2.Target_Angle >= 0)
//      lifting->Lifting_left_2.Target_Angle = 0;
////		
//		if(lifting->Lifting_right_2.Target_Angle <= 0)                        
//			lifting->Lifting_right_2.Target_Angle = 0;
//    else if(lifting->Lifting_right_2.Target_Angle >= 20000)
//      lifting->Lifting_right_2.Target_Angle = 20000;
//		
//		//转换为累加角度
//	  if((lifting->Motors3508.motor[2].Data.Angle<-100)&&(lifting->Lifting_left_2.angle_last>100))
//		{
//		  lifting->Lifting_left_2.angle +=360+lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[2].Data.Angle>100)&&(lifting->Lifting_left_2.angle_last<-100))
//		{
//		  lifting->Lifting_left_2.angle +=-360+lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_left_2.angle +=lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		 lifting->Lifting_left_2.angle_last=lifting->Motors3508.motor[2].Data.Angle;

//    
//		if((lifting->Motors3508.motor[3].Data.Angle<-100)&&(lifting->Lifting_right_2.angle_last>100))
//		{
//		  lifting->Lifting_right_2.angle +=360+lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[3].Data.Angle>100)&&(lifting->Lifting_right_2.angle_last<-100))
//		{
//		  lifting->Lifting_right_2.angle +=-360+lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_right_2.angle +=lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		 lifting->Lifting_right_2.angle_last=lifting->Motors3508.motor[3].Data.Angle;
//		
//		lifting->Motors3508.motor[2].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 5) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 4) , lifting->Lifting_left_2.Target_Angle, lifting->Lifting_left_2.angle)  ,lifting->Motors3508.motor[2].Data.SpeedRPM);
//		lifting->Motors3508.motor[3].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 7) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 6) , lifting->Lifting_right_2.Target_Angle, lifting->Lifting_right_2.angle)  , lifting->Motors3508.motor[3].Data.SpeedRPM);
// 
//    if(lifting->Motors3508.motor[2].Data.Output>=Motoroutputlimit_2)  lifting->Motors3508.motor[2].Data.Output = Motoroutputlimit_2;
//   	if(lifting->Motors3508.motor[2].Data.Output<=-Motoroutputlimit_2)  lifting->Motors3508.motor[2].Data.Output = -Motoroutputlimit_2;
//		if(lifting->Motors3508.motor[3].Data.Output>=Motoroutputlimit_2)  lifting->Motors3508.motor[3].Data.Output = Motoroutputlimit_2;
//   	if(lifting->Motors3508.motor[3].Data.Output<=-Motoroutputlimit_2)  lifting->Motors3508.motor[3].Data.Output = -Motoroutputlimit_2;

//		//写入电机数据
//		MotorFillData(&lifting->Motors3508.motor[2], lifting->Motors3508.motor[2].Data.Output);
//   	MotorFillData(&lifting->Motors3508.motor[3], lifting->Motors3508.motor[3].Data.Output);
//}


//void Lifting_Ore_secondpart_reset(Lifting_t *lifting,RC_Ctrl *rc_ctrl,float lifting_move_distance)
//{
//		float target_2;
//		target_2=lifting_move_distance*lifting_second_param;			//换算公式
//		int direction_flag_2=0;
//		
//		if(lifting->Lifting_right_2.Target_Angle<=target_2)
//		{
//			lifting->Lifting_left_2.Target_Angle -= 7;		//类似于缓启动
//			lifting->Lifting_right_2.Target_Angle += 7;
//			if(lifting->Lifting_right_2.Target_Angle>=target_2)
//			{
//				lifting->Lifting_right_2.Target_Angle=target_2;
//				lifting->Lifting_left_2.Target_Angle = -target_2;
//			}
//		}
//	
//		if(lifting->Lifting_right_2.Target_Angle>target_2)
//		{
//			lifting->Lifting_left_2.Target_Angle += 7;
//			lifting->Lifting_right_2.Target_Angle -= 7;
//			if(lifting->Lifting_right_2.Target_Angle<=target_2)
//			{
//				lifting->Lifting_right_2.Target_Angle=target_2;
//				lifting->Lifting_left_2.Target_Angle = -target_2;
//			}
//		}

//		//	运动范围限幅
//		if(lifting->Lifting_left_2.Target_Angle <= -20000) 
//			lifting->Lifting_left_2.Target_Angle = -20000;
//    else if(lifting->Lifting_left_2.Target_Angle >= 0)
//      lifting->Lifting_left_2.Target_Angle = 0;
////		
//		if(lifting->Lifting_right_2.Target_Angle <= 0)                        
//			lifting->Lifting_right_2.Target_Angle = 0;
//    else if(lifting->Lifting_right_2.Target_Angle >= 20000)
//      lifting->Lifting_right_2.Target_Angle = 20000;
//		
//		//转换为累加角度
//	  if((lifting->Motors3508.motor[2].Data.Angle<-100)&&(lifting->Lifting_left_2.angle_last>100))
//		{
//		  lifting->Lifting_left_2.angle +=360+lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[2].Data.Angle>100)&&(lifting->Lifting_left_2.angle_last<-100))
//		{
//		  lifting->Lifting_left_2.angle +=-360+lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_left_2.angle +=lifting->Motors3508.motor[2].Data.Angle-lifting->Lifting_left_2.angle_last;
//		}
//		 lifting->Lifting_left_2.angle_last=lifting->Motors3508.motor[2].Data.Angle;

//    
//		if((lifting->Motors3508.motor[3].Data.Angle<-100)&&(lifting->Lifting_right_2.angle_last>100))
//		{
//		  lifting->Lifting_right_2.angle +=360+lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		else if(( lifting->Motors3508.motor[3].Data.Angle>100)&&(lifting->Lifting_right_2.angle_last<-100))
//		{
//		  lifting->Lifting_right_2.angle +=-360+lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		else 
//		{
//		  lifting->Lifting_right_2.angle +=lifting->Motors3508.motor[3].Data.Angle-lifting->Lifting_right_2.angle_last;
//		}
//		 lifting->Lifting_right_2.angle_last=lifting->Motors3508.motor[3].Data.Angle;
//		
//		lifting->Motors3508.motor[2].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 5) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 4) , lifting->Lifting_left_2.Target_Angle, lifting->Lifting_left_2.angle)  ,lifting->Motors3508.motor[2].Data.SpeedRPM);
//		lifting->Motors3508.motor[3].Data.Output =BasePID_LiftingSpeedControl((BasePID_Object*)(lifting->Motors3508.turnPID + 7) , BasePID_LiftingAngleControl((BasePID_Object*)(lifting->Motors3508.turnPID + 6) , lifting->Lifting_right_2.Target_Angle, lifting->Lifting_right_2.angle)  , lifting->Motors3508.motor[3].Data.SpeedRPM);
// 
//    if(lifting->Motors3508.motor[2].Data.Output>=Motoroutputlimit)  lifting->Motors3508.motor[2].Data.Output = Motoroutputlimit;
//   	if(lifting->Motors3508.motor[2].Data.Output<=-Motoroutputlimit)  lifting->Motors3508.motor[2].Data.Output = -Motoroutputlimit;
//		if(lifting->Motors3508.motor[3].Data.Output>=12000)  lifting->Motors3508.motor[3].Data.Output = 12000;
//   	if(lifting->Motors3508.motor[3].Data.Output<=-12000)  lifting->Motors3508.motor[3].Data.Output = -12000;

//		//写入电机数据
//		MotorFillData(&lifting->Motors3508.motor[2], lifting->Motors3508.motor[2].Data.Output);
//   	MotorFillData(&lifting->Motors3508.motor[3], lifting->Motors3508.motor[3].Data.Output);
//}
