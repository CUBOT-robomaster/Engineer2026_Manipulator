#include <brain.h>
#include "driver_usart.h"
#include "hardware_config.h"
//CubotBrain_t Brain;
//uint8_t RobotToBrainTimeBuffer[21]__attribute__((at(0x24002160)));
// uint8_t RobotToBrainQuestBuffer_WorkingModel[5] __attribute__((at(0x24002210)));   //请求数据

//uint8_t RobotToBrainQuest=0;
//uint32_t count_robot_to_brain=0;
//uint32_t auto_ore_time=0;
//uint8_t ore_begin_flag=0;
//float sideway_parma=20.58;   //15368
//float protract_parma=-21.49;
//float lifting_parma=1;
//float servo1_parma=0.001;         //视觉乘以1000，这边除以1000变回弧度制
//float servo2_parma=180/3.1415926*0.001;
//float servo3_parma=180/3.1415926*0.001;

//uint8_t auto_ore_data_ok_flag=0;
//uint32_t auto_ore_delay=250;
//uint8_t last_ore_ok_flag;
//uint16_t delay_ore_time;
//uint8_t Brain_callback(uint8_t * recBuffer, uint16_t len)
//{
//  Brain_DataUnpack(&Brain,recBuffer,len);
//	return 0;
//}

//void Brain_DataUnpack(CubotBrain_t* brain, uint8_t* recBuffer,uint16_t len)
//{
//	 uint16_t i,j,k;
////	for(i=0;i<2000;i++)
////	{
////   if(recBuffer[0+27*i]==0xAA)
////	 {
////	   if(recBuffer[26+27*i]==0xDD)
////			brain->FrameCoreID=recBuffer[1+27*i];
////			brain->BrainCore[brain->FrameCoreID].Trajectory_msg.sideway_msg=recBuffer[2+27*i]<<24|recBuffer[3+27*i]<<16|recBuffer[4+27*i]<<8|recBuffer[5+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.lifting_msg=recBuffer[6+27*i]<<24|recBuffer[7+27*i]<<16|recBuffer[8+27*i]<<8|recBuffer[9+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.protract_msg=recBuffer[10+27*i]<<24|recBuffer[11+27*i]<<16|recBuffer[12+27*i]<<8|recBuffer[13+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Yaw_msg=recBuffer[14+27*i]<<24|recBuffer[15+27*i]<<16|recBuffer[16+27*i]<<8|recBuffer[17+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Pitch_msg=recBuffer[18+27*i]<<24|recBuffer[19+27*i]<<16|recBuffer[20+27*i]<<8|recBuffer[21+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Roll_msg=recBuffer[22+27*i]<<24|recBuffer[23+27*i]<<16|recBuffer[24+27*i]<<8|recBuffer[25+27*i];
////	 
////	 }
////	 if(recBuffer[0+27*i]==0xAA)
////	 {
////	   if(recBuffer[26+27*i]==0xCC)
////			brain->FrameCoreID=recBuffer[1+27*i];
////			brain->BrainCore[brain->FrameCoreID].Trajectory_msg.sideway_msg=recBuffer[2+27*i]<<24|recBuffer[3+27*i]<<16|recBuffer[4+27*i]<<8|recBuffer[5+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.lifting_msg=recBuffer[6+27*i]<<24|recBuffer[7+27*i]<<16|recBuffer[8+27*i]<<8|recBuffer[9+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.protract_msg=recBuffer[10+27*i]<<24|recBuffer[11+27*i]<<16|recBuffer[12+27*i]<<8|recBuffer[13+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Yaw_msg=recBuffer[14+27*i]<<24|recBuffer[15+27*i]<<16|recBuffer[16+27*i]<<8|recBuffer[17+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Pitch_msg=recBuffer[18+27*i]<<24|recBuffer[19+27*i]<<16|recBuffer[20+27*i]<<8|recBuffer[21+27*i];
////		  brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Roll_msg=recBuffer[22+27*i]<<24|recBuffer[23+27*i]<<16|recBuffer[24+27*i]<<8|recBuffer[25+27*i];
////		  brain->auto_ore_flag=1;
////		  brain->sum_msg=brain->FrameCoreID;
////	 }
//// } 
//	 for(i=0;i<2000;i++)
//	{
//   if(recBuffer[0+27*i]==0xAA)
//	 {
//		if(recBuffer[26+27*i]==0xDD)
//			{
//					brain->FrameCoreID=recBuffer[1+27*i];
//					for(j=0;j<6;j++)
//				 {
//					brain->data_u.data_b[0]=recBuffer[2+27*i+4*j];
//					brain->data_u.data_b[1]=recBuffer[3+27*i+4*j];
//					brain->data_u.data_b[2]=recBuffer[4+27*i+4*j];    //sideway_msg //lifting_msg//protract_msg//Yaw_msg//Pitch_msg
//					brain->data_u.data_b[3]=recBuffer[5+27*i+4*j];
//				  if(j==0)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.protract_msg=brain->data_u.data;
//          if(j==1)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.sideway_msg=brain->data_u.data; 
//					if(j==2)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.lifting_msg=brain->data_u.data;  
//					if(j==3)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Yaw_msg=brain->data_u.data;  
//					if(j==4)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Pitch_msg=brain->data_u.data;  
//					if(j==5)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Roll_msg=brain->data_u.data;  		//Roll_msg			  
//			   }
//		  }
//	 }
//	 if(recBuffer[0+27*i]==0xAA)
//	 {
//		 if(recBuffer[26+27*i]==0xDD)
//			{
//				if(recBuffer[27+27*i]==0xCC)
//				{
//					brain->FrameCoreID=recBuffer[1+27*i];
//					for(j=0;j<6;j++)
//				 {
//					brain->data_u.data_b[0]=recBuffer[2+27*i+4*j];
//					brain->data_u.data_b[1]=recBuffer[3+27*i+4*j];
//					brain->data_u.data_b[2]=recBuffer[4+27*i+4*j];
//					brain->data_u.data_b[3]=recBuffer[5+27*i+4*j];
//				  if(j==0)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.protract_msg=brain->data_u.data;
//          if(j==1)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.sideway_msg=brain->data_u.data; 
//					if(j==2)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.lifting_msg=brain->data_u.data;  
//					if(j==3)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Yaw_msg=brain->data_u.data;  
//					if(j==4)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Pitch_msg=brain->data_u.data;  
//					if(j==5)brain->BrainCore[brain->FrameCoreID].Trajectory_msg.Roll_msg=brain->data_u.data;					 
//			   }
//				 brain->auto_ore_flag=1;
//		     brain->sum_msg=brain->FrameCoreID;
//				 break;
//			 }
//		  }
//	   
//	 }
//  }
//	if(recBuffer[0]==0xaa)
//{
//    for(k=1;k<20;k++)
//	{
//	   if(recBuffer[k]==0xff)
//		 {
//		   brain->auto_ore_flag=0;
//		   brain->error=1;
//		 }
//		 else brain->error=0;
//	}
//}
//// 
////if(recBuffer[0]=)
////	 for(i=0;i<=26;i++)
////	 {
////	   if(recBuffer[i]==0xff)
////			brain->error=1;
////		 else brain->error=0; 
////	 }
//}
//void Request_identification()
//{
//	if(RobotToBrainQuest==0)
//	{
//		count_robot_to_brain++;
//		if(count_robot_to_brain==5)
//		{
//			RobotToBrainQuestBuffer_WorkingModel[0] = 0xff;
//			RobotToBrainQuestBuffer_WorkingModel[1] = 0xAA;        //固定为0x02
//			RobotToBrainQuestBuffer_WorkingModel[2] = 0xFF;      //机器人大脑内核编号，目前固定为0x01
//			RobotToBrainQuestBuffer_WorkingModel[3] = 0xDD;
//			RobotToBrainQuestBuffer_WorkingModel[4] = 0xDD;
//			HAL_UART_Transmit_DMA(&huart4, RobotToBrainQuestBuffer_WorkingModel, 1);
//		}
//	}
//	else count_robot_to_brain=0;

//}	
//void auto_ore_assignment(Servo_t *servo,CubotBrain_t* brain)
//{
//	if(brain->auto_ore_flag_ok==1)
//	{
//		TIM3->CCR4=1900;
//		if(auto_ore_time%auto_ore_delay==0&&auto_ore_data_ok_flag==0)
//		{	
//	  servo->oremsg.lifting_dis =brain->BrainCore[auto_ore_time/auto_ore_delay].Trajectory_msg.lifting_msg*lifting_parma;
//	  servo->oremsg.pitch2_dis  =brain->BrainCore[auto_ore_time/auto_ore_delay].Trajectory_msg.Pitch_msg*servo2_parma;
//	  servo->oremsg.protract_dis=brain->BrainCore[auto_ore_time/auto_ore_delay].Trajectory_msg.protract_msg;//*protract_parma
//	  servo->oremsg.roll3_dis   =brain->BrainCore[auto_ore_time/auto_ore_delay].Trajectory_msg.Roll_msg*servo3_parma;
//	  servo->oremsg.sideway_dis =(brain->BrainCore[auto_ore_time/auto_ore_delay].Trajectory_msg.sideway_msg);//*sideway_parma
//	  servo->oremsg.yall1_dis   =brain->BrainCore[auto_ore_time/auto_ore_delay].Trajectory_msg.Yaw_msg*servo1_parma;
//		if(auto_ore_time/auto_ore_delay==0)
//		{
//		  servo->oremsg.Vx_sideway=servo->oremsg.sideway_dis*sideway_parma;
//			servo->oremsg.Vy_protract=servo->oremsg.protract_dis*protract_parma;
//		
//		}
//		if(auto_ore_time/auto_ore_delay>=1)
//		{
//		  servo->oremsg.Vx_sideway=(servo->oremsg.sideway_dis-brain->BrainCore[auto_ore_time/auto_ore_delay-1].Trajectory_msg.sideway_msg)*sideway_parma;
//			servo->oremsg.Vy_protract=(servo->oremsg.protract_dis-brain->BrainCore[auto_ore_time/auto_ore_delay-1].Trajectory_msg.protract_msg)*protract_parma;
//		}
//		
//    ore_begin_flag=1;			
//		}
//		if(auto_ore_time/auto_ore_delay==brain->sum_msg||last_ore_ok_flag==1)
//		{
//			last_ore_ok_flag=1;
//			if(last_ore_ok_flag==1)
//			{
//			   delay_ore_time++;
//			   if(delay_ore_time>=auto_ore_delay)
//				 {
//				    delay_ore_time=0;
//				    brain->auto_ore_flag_ok=0;
//					 last_ore_ok_flag=0;
//				 }
//			}
//			auto_ore_data_ok_flag=1;
//		}
//		
//		auto_ore_time++;
//	}
//	else 
//	{
//	   auto_ore_time=0;
//		auto_ore_data_ok_flag=0;
//	}
//}
