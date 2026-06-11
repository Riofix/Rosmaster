/**
 * @file bsp_usart.h
 * @brief Bootloader 串口驱动 — 仅 USART2, RXNE 中断 + 阻塞发送
 */

#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "stm32f10x.h"

/* USART2 接收缓冲区大小 (256B/块 + 1ms 间隔, 512 兜两倍) */
#define USART2_RX_BUFFER_SIZE  512

/* ========== USART2 接口 ========== */
void USART2_Init(uint32_t baudrate);

void USART2_SendByte(uint8_t byte);
void USART2_SendBuffer(uint8_t *buffer, uint16_t len);

uint16_t USART2_Available(void);
uint8_t  USART2_ReadByte(void);
uint16_t USART2_Read(uint8_t *buf, uint16_t len);
void     USART2_Flush(void);

#endif /* _BSP_USART_H */
