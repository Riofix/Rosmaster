#ifndef __APP_H
#define __APP_H

#include <stdint.h>

/**
 * @brief 系统运行状态上下文
 */
typedef struct
{
    uint8_t oled_mode;         // 0: 待机/Logo, 1: 显示MPU数据, 2: 显示电机数据
    uint8_t mpu_stream;        // MPU自动上报标志位 0：关闭 1：开启
    uint8_t stepmotor_stream;  // 电机数据上报标志位 0：关闭 1：开启
    uint8_t pwm_state_stream;  // PWM数据上报标志位 0：关闭 1：开启
    uint8_t rgb_serson_stream; // 颜色传感器自动上报标志位 0：关闭 1：开启
    uint8_t system_ready;      // 系统初始化完成标志
} App_Context_t;

extern App_Context_t g_app_context;

/**
 * @brief App 层统一初始化
 */
void App_Init(void);

/**
 * @brief App 层周期性调度逻辑 (10ms)
 */
void App_Tick(void);

#endif
