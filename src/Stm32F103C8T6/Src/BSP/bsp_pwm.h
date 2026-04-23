#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "stm32f10x.h"
#include <stdint.h>

/* PWM Channel Descriptor Table Driver
 * --------------------------------------
 * CH1: PA6 -> TIM3_CH1 -> BLDC motor (10kHz)
 * CH2: PB8 -> TIM4_CH3 -> Servo 1     (50Hz)
 * CH3: PB9 -> TIM4_CH4 -> Servo 2     (50Hz)
 * -------------------------------------- */

void PWM_Bldc_Init(void);
void PWM_Servo_Init(void);
void PWM_Bldc_SetDuty(uint8_t duty);
void PWM_Bldc_Stop(void);
void PWM_Servo_SetAngle(uint8_t ch, uint8_t angle); // 0 - 180
static uint16_t Angle_To_Pulse(float angle);

#endif /* __BSP_PWM_H */
