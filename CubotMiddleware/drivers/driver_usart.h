#ifndef DRV_USART_H
#define DRV_USART_H

#include "stm32h7xx_hal.h"



typedef uint8_t (*UART_RxIdleCallback)(uint8_t *rxBuffer, uint16_t size); ///<相当于声明了一个函数指针类型

typedef struct{
    uint8_t*  Data;
	uint16_t Size;
}UART_RxBuffer;

typedef struct 
{
	UART_HandleTypeDef* Handle;
	UART_RxIdleCallback RxIdleCallback;        
}UART_Object;



void UART_Receive_DMA(UART_Object* uart, UART_RxBuffer* rxBuffer);
void UART_ENABLE_IT(UART_Object* uart, UART_RxBuffer* rxBuffer);
void UARTx_Init(UART_HandleTypeDef* handle, UART_RxIdleCallback rxIdleCallback);
void UART_Idle_Handler(UART_Object* uart, UART_RxBuffer* rxBuffer);
void UART_IT_Handler(UART_Object* uart, UART_RxBuffer* rxBuffer);
void UART_Receive_IT(UART_Object* uart, UART_RxBuffer* rxBuffer);
extern UART_Object uart1;
extern UART_Object uart2;
extern UART_Object uart3;
extern UART_Object uart4;
extern UART_Object uart5;
extern UART_Object uart6;
extern UART_Object uart7;
extern UART_Object uart8;
#endif












