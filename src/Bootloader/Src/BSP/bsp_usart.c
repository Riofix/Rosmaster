/**
 * @file bsp_usart.c
 * @brief Bootloader 串口驱动 — USART2 (PA2/PA3), DMA Circular + 阻塞发送
 * @note  使用 DMA 而非 RXNE 中断：Flash 擦写会暂停 CPU，
 *        DMA 硬件独立搬运，不会丢数据。
 */

#include "bsp_usart.h"
#include <string.h>

/* ========== 内部静态缓冲与指针 ========== */
static uint8_t  usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
static volatile uint16_t rx2_write_idx = 0;
static uint16_t rx2_read_idx  = 0;

/* ========== USART2 初始化 (DMA Circular, PA2=TX, PA3=RX) ========== */
void USART2_Init(uint32_t baudrate) {
    /* 1. 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* 2. GPIO: PA2=TX(AF_PP), PA3=RX(IPU) */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 3. USART 参数: 8N1 */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    /* 4. DMA1_Channel6: USART2_RX → 环缓, Circular 模式 */
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)usart2_rx_buffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize         = USART2_RX_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel6, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

    /* 5. IDLE 中断 — 检测数据流停顿, 更新写指针 */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

/* ========== USART2 IDLE 中断 ========== */
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
        volatile uint32_t tmp = USART2->SR;   /* 读 SR */
        tmp = USART2->DR;                     /* 读 DR 清除 IDLE 标志 */
        (void)tmp;
        rx2_write_idx = USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
    }
}

/* ========== 阻塞发送 ========== */
void USART2_SendByte(uint8_t data) {
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, data);
}

void USART2_SendBuffer(uint8_t *buffer, uint16_t len) {
    while (len--) {
        USART2_SendByte(*buffer++);
    }
}

/* ========== 环形缓冲读取 ========== */
uint16_t USART2_Available(void) {
    /* 同步最新的 DMA 写入位置 */
    uint16_t write_idx = USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
    rx2_write_idx = write_idx;

    if (write_idx >= rx2_read_idx) {
        return write_idx - rx2_read_idx;
    } else {
        return USART2_RX_BUFFER_SIZE - rx2_read_idx + write_idx;
    }
}

uint8_t USART2_ReadByte(void) {
    uint8_t data = 0;
    uint16_t write_idx = USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
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

void USART2_Flush(void) {
    rx2_read_idx = USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
}
