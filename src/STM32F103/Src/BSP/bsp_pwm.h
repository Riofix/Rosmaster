#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "stm32f10x.h"
#include <stdint.h>

/* PWM Channel Descriptor Table Driver
 * CH1: PA6 -> TIM3_CH1 -> BLDC motor
 * CH2: PB0 -> TIM3_CH3 -> Servo 1
 * CH3: PB1 -> TIM3_CH4 -> Servo 2
 * Timer: 72MHz/(71+1)=1MHz, 1tick=1us, Period=20000us=50Hz */

typedef enum {
    PWM_CH1    = 0,
    PWM_CH2    = 1,
    PWM_CH3    = 2,
    PWM_CH_MAX = 3
} PWM_Channel_t;

#define PWM_PERIOD_US    20000u
#define PWM_SERVO_MIN_US 500u
#define PWM_SERVO_MAX_US 2500u

void PWM_Init(void);
void PWM_SetDuty(PWM_Channel_t ch, uint8_t duty);
void PWM_SetPulse_us(PWM_Channel_t ch, uint16_t pulse_us);
void PWM_Servo_SetAngle(PWM_Channel_t ch, uint8_t angle);
void PWM_Stop(PWM_Channel_t ch);

#endif /* __BSP_PWM_H */
