#include "interaction_image.h"
#include "string.h"
#include "usart.h"
#include "vt13.h"
#include "check.h"
#include "control_logic.h"
#include "filter.h"
custom_robot_data_t Custom;//自定义控制器

int16_t recv_test = 0;

int16_t Joint0_right,Joint1_right,Joint2_right,Joint3_right,Joint4_right,Joint5_right,Joint6_right;
int16_t Joint0_left,Joint1_left,Joint2_left,Joint3_left,Joint4_left,Joint5_left,Joint6_left;

unsigned char Usart2_RxBuffer[USART2_RXBUF_SIZE]__attribute__((at(0x24006400)));
unsigned char Usart2_TxBuffer[USART2_TXBUF_SIZE]__attribute__((at(0x24006800)));

UART_RxBuffer uart2_buffer={
	.Data = Usart2_RxBuffer,
	.Size = USART2_RXBUF_SIZE
};
uint8_t ref_image_packge[10][40];  //最多一次收50个包
/**
  * @brief  图传接收数据,测试,在定时中断才处理数据
	*/
void imagetrans_recv_datas_copy(uint8_t *pdata,uint8_t * pBuffer)//只复制
{
	memcpy(pBuffer,pdata,USART2_RXBUF_SIZE);
}
void controller_recv_datas_modify(uint8_t *pdata,custom_robot_data_t* custom)
{  
	/*自定义控制器（工程）*/
	uint16_t cmd_id;
	cmd_id=*(pdata+6)<<8|*(pdata+5); 
	if(cmd_id==0x0302){	
		uint8_t switch_zero,switch_one;
		
		check_robot_state.usart_state.Check_custom = 0;
		
		/* 右手J0和J1相反，J4和J6相反 */
		Joint1_right=*(pdata+7)|*(pdata+8)<<8;
		Joint0_right=*(pdata+9)|*(pdata+10)<<8;
		Joint2_right=*(pdata+11)|*(pdata+12)<<8;
		Joint3_right=*(pdata+13)|*(pdata+14)<<8;
		Joint6_right=*(pdata+15)|*(pdata+16)<<8;
		Joint5_right=*(pdata+17)|*(pdata+18)<<8;
		Joint4_right=*(pdata+19)|*(pdata+20)<<8;
		
		/* 左手J0和J1相反，J2和J3相反，J4和J6相反 */
		Joint1_left=*(pdata+21)|*(pdata+22)<<8;
		Joint0_left=*(pdata+23)|*(pdata+24)<<8;
		Joint3_left=*(pdata+25)|*(pdata+26)<<8;
		Joint2_left=*(pdata+27)|*(pdata+28)<<8;
		Joint6_left=*(pdata+29)|*(pdata+30)<<8;
		Joint5_left=*(pdata+31)|*(pdata+32)<<8;
		Joint4_left=*(pdata+33)|*(pdata+34)<<8;
		
		switch_zero = *(pdata+35);
		switch_one = *(pdata+36);
		
		/* 直接将提取出的 0 或 1 赋值给数组 */
		for (int i = 0; i < 4; i++) {
			custom->image_recv.Coordinate.switches[i] = (switch_zero >> i) & 0x01;
		}
		
		custom->image_recv.Coordinate.cc_circle=*(pdata+32)&0x01;

		/* 未经滤波处理的数据 */
		custom->image_recv.Coordinate.joint0_right=(float)Joint0_right/50;
		custom->image_recv.Coordinate.joint1_right=(float)Joint1_right/50;
		custom->image_recv.Coordinate.joint2_right=(float)Joint2_right/50;
		custom->image_recv.Coordinate.joint3_right=(float)Joint3_right/50;
		custom->image_recv.Coordinate.joint4_right=(float)Joint4_right/50;
		custom->image_recv.Coordinate.joint5_right=(float)Joint5_right/50;
		custom->image_recv.Coordinate.joint6_right=(int8_t)Joint6_right;

		custom->image_recv.Coordinate.joint0_left=(float)Joint0_left/50;
		custom->image_recv.Coordinate.joint1_left=(float)Joint1_left/50;
		custom->image_recv.Coordinate.joint2_left=(float)Joint2_left/50;
		custom->image_recv.Coordinate.joint3_left=(float)Joint3_left/50;
		custom->image_recv.Coordinate.joint4_left=(float)Joint4_left/50;
		custom->image_recv.Coordinate.joint5_left=(float)Joint5_left/50;
		custom->image_recv.Coordinate.joint6_left=(int8_t)Joint6_left;		//Joint6数据为1是正转，数据为-1时逆转，数据为0时不动
		/* 滤波函数不要写在图传接收函数里，不然帧率过低很影响效果 */
		
		controller_count = 0;
	}
}

uint8_t image_command(uint8_t *data, uint16_t len) 
{
   
    static uint32_t Verify_CRC8_OK;
    static uint32_t Verify_CRC16_OK;
    uint16_t i = 0;
    uint16_t pack_size = 0;

    while (pack_size < len) 
	{
		if (*data != 0xA5) 
		{
			data++;
			pack_size++;
			continue;
		}

		// 检查剩余数据是否足够解析头部
		if (pack_size + 7 > len) 
			break;

		// 解析 data_len（避免非对齐访问）
		uint16_t data_len;
		memcpy(&data_len, data + 1, 2);
		Custom.image_send.image_frame_header.data_length = data_len;

		// 检查数据包完整性
		uint16_t total_packet_size = 7 + data_len + 2;
		if (pack_size + total_packet_size > len)
			break;

		// 校验 CRC
		Verify_CRC8_OK = Verify_CRC8_Check_Sum(data, frame_header_len);
		Verify_CRC16_OK = Verify_CRC16_Check_Sum(data, total_packet_size);

		if (Verify_CRC8_OK && Verify_CRC16_OK) 
		{
			if (i < 200) 
			{
				memcpy(ref_image_packge[i], data, total_packet_size);
				controller_recv_datas_modify(ref_image_packge[i],&Custom);
				i++;
				if(i>25)
					i=25;
			}
		}

		data += total_packet_size;
		pack_size += total_packet_size;

		if (pack_size >= 150)
			break;
    }
    return 0;
}

	
/*接收回调*/
uint8_t imagetrans_callback(uint8_t * recBuffer, uint16_t len)
{
	controller_recv_datas_modify(recBuffer,&Custom);	
	vt13_recv_datas_modify(recBuffer,&vT13);
//	imagetrans_recv_datas_copy(Usart2_RxBuffer,recv_Buffer);//测试,在定时中断才处理数据
	return 0;
}

