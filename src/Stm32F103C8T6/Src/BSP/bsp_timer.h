#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "stm32f10x.h"
#include <stdint.h>

/**
 * @brief  初始化 TIM2 为自由运行微秒计数器
 * @note   72MHz / 7200 = 10kHz, 每 100µs +1, 约 6.55 秒溢出回绕
 */
void DtTimer_Init(void);

/**
 * @brief  读取当前微秒计数值
 * @return 自初始化以来的累计微秒数
 */
uint32_t DtTimer_GetUs(void);

#endif /* __BSP_TIMER_H */
