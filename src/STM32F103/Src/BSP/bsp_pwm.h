#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "stm32f10x.h"

void PWM_Init(void);                // 初始化PWM
void PWM_SetDuty(uint8_t duty);     // 设置占空比 0-100

#endif
