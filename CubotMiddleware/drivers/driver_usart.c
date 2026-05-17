#include "driver_usart.h"
#include "interaction_image.h"

UART_Object uart1;
UART_Object uart2;
UART_Object uart3;
UART_Object uart4;
UART_Object uart5;
UART_Object uart6;
UART_Object uart7;
UART_Object uart8;

void UARTx_Init(UART_HandleTypeDef* handle, UART_RxIdleCallback rxIdleCallback)
{
    if(handle->Instance==USART1)
		{
		   uart1.Handle=handle;
			 uart1.RxIdleCallback=rxIdleCallback;
		}
		if(handle->Instance==USART2)
		{
		   uart2.Handle=handle;
			 uart2.RxIdleCallback=rxIdleCallback;
		}
		if(handle->Instance==USART3)
		{
		   uart3.Handle=handle;
			 uart3.RxIdleCallback=rxIdleCallback;
		}
		
			// 初始化uart4
	  if (handle->Instance == UART4)
	  {
	  	uart4.Handle = handle;
	  	uart4.RxIdleCallback = rxIdleCallback;
  	}
	
	// 初始化uart5
  	if (handle->Instance == UART5)
	  {
	  	uart5.Handle = handle;
	  	uart5.RxIdleCallback = rxIdleCallback;
	  }
	
	// 初始化uart6
	  if (handle->Instance == USART6)
	  {
	  	uart6.Handle = handle;
	  	uart6.RxIdleCallback = rxIdleCallback;
	  }
		if (handle->Instance == UART7)
	  {
	  	uart7.Handle = handle;
	  	uart7.RxIdleCallback = rxIdleCallback;
	  }
		if (handle->Instance == UART8)
	  {
	  	uart8.Handle = handle;
	  	uart8.RxIdleCallback = rxIdleCallback;
	  }
		

}

void UART_ENABLE_IT(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
	//HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
	__HAL_UART_ENABLE_IT(uart->Handle, UART_IT_IDLE);   		//<开启中断前分配回调比较好
} 


void UART_Receive_DMA(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
   HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
}

void UART_Receive_IT(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
   HAL_UART_Receive_IT(uart->Handle, rxBuffer->Data, rxBuffer->Size);
}
/**
  * @brief  串口设备中断函数，执行中断DMA操作，调用串口用户回调函数 
  */
void UART_Idle_Handler(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
	assert_param(uart != NULL);
	uint16_t usart_rx_num;
	
	if((__HAL_UART_GET_FLAG(uart->Handle, UART_FLAG_IDLE) != RESET))
	{	
		HAL_UART_DMAStop(uart->Handle);																																					//< 关闭DMA，防止解算过程中数据更新，造成丢失数据															
		__HAL_UART_CLEAR_IDLEFLAG(uart->Handle);																																//< 清除idle标志位，防止再次进入中断		
		__HAL_UART_CLEAR_OREFLAG(uart->Handle);	
		usart_rx_num = rxBuffer->Size - ((DMA_Stream_TypeDef*)uart->Handle->hdmarx->Instance)->NDTR; 
//		if ((((DMA_Stream_TypeDef*)uartManage->huart.hdmarx->Instance)->NDTR) == uartManage->rxBufferSize)		//< 判断缓存区长度 是否等于 DMA中未传输数据长度。若为等于，数据更新完一次。 
		{																																																				//< 如果发送方先启动，则接收方开始接收后是否需要匹配头字节和尾字节																																														
			if((*uart).RxIdleCallback!=NULL)
			  uart->RxIdleCallback(rxBuffer->Data, usart_rx_num);            				 		   												//<用户回调
		}	
		HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
	}
}

void UART_IT_Handler(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
//	if((*uart).RxIdleCallback!=NULL)
//			  uart->RxIdleCallback(rxBuffer->Data, rxBuffer->Size);
	HAL_UART_Receive_IT(uart->Handle, rxBuffer->Data, rxBuffer->Size);
}

