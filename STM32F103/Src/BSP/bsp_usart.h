#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "stm32f10x.h"
#include <string.h>

#define USART1_RX_BUFFER_SIZE 128
#define USART2_RX_BUFFER_SIZE 128

// USART1 接口
void USART1_Init(uint32_t baudrate);
void USART1_SendByte(uint8_t byte);
void USART1_SendBuffer(uint8_t* buffer, uint16_t len);
//uint16_t USART1_GetRxWriteIndex(void);
//uint8_t* USART1_GetRxBuffer(void);
uint8_t USART1_GetRxData(uint8_t *pDest);

// USART2 接口 
void USART2_Init(uint32_t baudrate);
void USART2_SendByte(uint8_t byte);
void USART2_SendBuffer(uint8_t* buffer, uint16_t len);
uint16_t USART2_GetRxWriteIndex(void);
uint8_t* USART2_GetRxBuffer(void);

#endif
