/**
 * @file bsp_usart.h
 * @brief 串口硬件驱动头文件
 * @details 定义底层USART外设及其环形缓冲区对外接口
 */
#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "stm32f10x.h"

// 缓冲区大小定义
#define USART1_RX_BUFFER_SIZE 128
#define USART2_RX_BUFFER_SIZE 256 // 应对 ESP8266 突发网络数据

// ====== USART1 接口 (针对 Emm_V5，RXNE中断 + 发送阻塞) ======
void USART1_Init(uint32_t baudrate);
void USART1_SendByte(uint8_t byte);
void USART1_SendBuffer(uint8_t *buffer, uint16_t len);

// 环形缓冲流式读取接口
uint16_t USART1_Available(void);
uint8_t USART1_ReadByte(void);
uint16_t USART1_Read(uint8_t *buf, uint16_t len);

// ====== USART2 接口 (针对 ESP8266，DMA Circular + 发送阻塞) ======
void USART2_Init(uint32_t baudrate);
void USART2_SendByte(uint8_t byte);
void USART2_SendBuffer(uint8_t *buffer, uint16_t len);

// 环形缓冲流式读取接口
uint16_t USART2_Available(void);
uint8_t USART2_ReadByte(void);
uint16_t USART2_Read(uint8_t *buf, uint16_t len);

#endif
