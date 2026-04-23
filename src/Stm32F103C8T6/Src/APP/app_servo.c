#include "app_servo.h"
#include "bsp_pwm.h" // 引入底层驱动
#include <string.h>

Servo_t g_servos[SERVO_NUM];

void App_Servo_Init(void)
{
    PWM_Servo_Init(); // 初始化PWM
}

// 设置舵机角度，地址位0时为广播设置
void App_Servo_SetAngle(uint8_t servo_id, uint8_t angle)
{
    if (servo_id < SERVO_NUM)
    {
        g_servos[servo_id].angle = angle;
        PWM_Servo_SetAngle(servo_id + 1, angle); // 设置PWM占空比
    }
    if (servo_id == 0)
    {
        g_servos[0].angle = angle;
        g_servos[1].angle = angle;
        PWM_Servo_SetAngle(1, angle);
        PWM_Servo_SetAngle(2, angle); // 设置PWM占空比
    }
}
