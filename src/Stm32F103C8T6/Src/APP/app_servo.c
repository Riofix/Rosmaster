#include "app_servo.h"
#include "bsp_pwm.h" // 引入底层驱动
#include <string.h>

Servo_t g_servos[SERVO_NUM];

void App_Servo_Init(void)
{
    PWM_Servo_Init();
	/* 上电默认 0° (开)，这个后续需要更改为关即90° */
    g_servos[0].angle = 0;
    g_servos[1].angle = 0;
    PWM_Servo_SetAngle(1, 0);
    PWM_Servo_SetAngle(2, 0);
}

// servo_id: 0=广播, 1=舵机1(PB8), 2=舵机2(PB9)
void App_Servo_SetAngle(uint8_t servo_id, uint8_t angle)
{
    if (servo_id == 0)
    {
        /* 广播: 两个舵机同时设置 */
        g_servos[0].angle = angle;
        g_servos[1].angle = angle;
        PWM_Servo_SetAngle(1, angle);
        PWM_Servo_SetAngle(2, angle);
    }
    else if (servo_id <= SERVO_NUM)
    {
        /* 单通道: 1→g_servos[0], PWM1; 2→g_servos[1], PWM2 */
        g_servos[servo_id - 1].angle = angle;
        PWM_Servo_SetAngle(servo_id, angle);
    }
}
