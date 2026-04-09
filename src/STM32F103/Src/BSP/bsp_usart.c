/**
 * @file bsp_usart.c
 * @brief 串口硬件驱动视窗源文件
 * @details 实现串口环形缓冲读写机制。USART1使用RXNE，USART2使用DMA Circular。
 */
#include "bsp_usart.h"
#include <string.h>

// ================= 内部静态缓冲与指针 =================
static uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
static volatile uint16_t rx1_write_idx = 0;
static uint16_t rx1_read_idx = 0;

static uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
// USART2 的 write_idx 会跟据 DMA 计数动态计算或在 IDLE 更新，增加安全性
static volatile uint16_t rx2_write_idx = 0;
static uint16_t rx2_read_idx = 0;

/* ================= USART1 配置 (下位机普通透传，RXNE 中断) =================
 */
void USART1_Init(uint32_t baudrate) {
  // 1. 开启时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  // 2. GPIO配置 (TX=PA9, RX=PA10)
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 3. USART1初始化
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_Init(USART1, &USART_InitStructure);

  // 4. 中断配置 (RXNE 中断取代 DMA)
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);
}

// USART1 中断服务函数
void USART1_IRQHandler(void) {
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    // 读取数据，这一步同时清除了 RXNE 标志位
    uint8_t data = USART_ReceiveData(USART1);

    // 放入环形缓冲区
    usart1_rx_buffer[rx1_write_idx] = data;
    rx1_write_idx = (rx1_write_idx + 1) % USART1_RX_BUFFER_SIZE;

    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

// ============== USART1 读写接口 ==============
void USART1_SendByte(uint8_t data) {
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    ;
  USART_SendData(USART1, data);
}

void USART1_SendBuffer(uint8_t *buffer, uint16_t len) {
  while (len--)
    USART1_SendByte(*buffer++);
}

uint16_t USART1_Available(void) {
  uint16_t write_idx = rx1_write_idx; // 获取快照，避免被中断打断
  if (write_idx >= rx1_read_idx) {
    return write_idx - rx1_read_idx;
  } else {
    return USART1_RX_BUFFER_SIZE - rx1_read_idx + write_idx;
  }
}

uint8_t USART1_ReadByte(void) {
  uint8_t data = 0;
  if (rx1_read_idx != rx1_write_idx) {
    data = usart1_rx_buffer[rx1_read_idx];
    rx1_read_idx = (rx1_read_idx + 1) % USART1_RX_BUFFER_SIZE;
  }
  return data;
}

uint16_t USART1_Read(uint8_t *buf, uint16_t len) {
  uint16_t count = 0;
  while (count < len && USART1_Available() > 0) {
    buf[count++] = USART1_ReadByte();
  }
  return count;
}

/* ================= USART2 配置 (上位机通信，DMA Circular) ================= */
void USART2_Init(uint32_t baudrate) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // TX: PA2
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // RX: PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_Init(USART2, &USART_InitStructure);

  // USART2_RX 对应 DMA1_Channel6，循环模式 (Circular)
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart2_rx_buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = USART2_RX_BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel6, ENABLE);
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

  // 开启 IDLE 中断，作为实时更新 write_idx 的辅助手段
  NVIC_InitTypeDef NVIC_InitStructure2;
  NVIC_InitStructure2.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure2);

  USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
  USART_Cmd(USART2, ENABLE);
}

// USART2 中断服务函数
void USART2_IRQHandler(void) {
  if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
    volatile uint32_t tmp = USART2->SR;
    tmp = USART2->DR; // 清除 IDLE 标志位
    (void)tmp;
    // 更新当前 DMA 已写入的位置
    rx2_write_idx =
        USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
  }
}

// ============== USART2 读写接口 ==============
void USART2_SendByte(uint8_t data) {
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    ;
  USART_SendData(USART2, data);
}

void USART2_SendBuffer(uint8_t *buffer, uint16_t len) {
  while (len--)
    USART2_SendByte(*buffer++);
}

uint16_t USART2_Available(void) {
  // 动态获取当前的 write_idx，防止 DMA 还在运转但 IDLE 还没触发导致数据滞留延迟
  uint16_t write_idx =
      USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
  rx2_write_idx = write_idx; // 顺便同步上

  if (write_idx >= rx2_read_idx) {
    return write_idx - rx2_read_idx;
  } else {
    return USART2_RX_BUFFER_SIZE - rx2_read_idx + write_idx;
  }
}

uint8_t USART2_ReadByte(void) {
  uint8_t data = 0;
  // 双重确认当前的最新写指针
  uint16_t write_idx =
      USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
  if (rx2_read_idx != write_idx) {
    data = usart2_rx_buffer[rx2_read_idx];
    rx2_read_idx = (rx2_read_idx + 1) % USART2_RX_BUFFER_SIZE;
  }
  return data;
}

uint16_t USART2_Read(uint8_t *buf, uint16_t len) {
  uint16_t count = 0;
  while (count < len && USART2_Available() > 0) {
    buf[count++] = USART2_ReadByte();
  }
  return count;
}

void USART_All_Init(uint32_t baudrate1, uint32_t baudrate2) {
  USART1_Init(baudrate1);
  USART2_Init(baudrate2);
}
