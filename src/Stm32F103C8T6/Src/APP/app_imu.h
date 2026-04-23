#ifndef _APP_IMU_H
#define _APP_IMU_H

#include "stm32f10x.h"
#include "bsp_mpu6050.h"
#include <stdio.h>

/* ---------------------------------------------------------
 * 数据封包参数定义
 * 将浮点角度放大 100 倍转换为 int16_t，保留两位小数精度
 * 范围: -180.00 ~ +180.00  ->  -18000 ~ +18000
 * --------------------------------------------------------- */
#define IMU_ANGLE_SCALE 100.0f

/**
 * @brief 用于数据包传输的 IMU 数据结构 (全部为 int16_t 整型)
 */
typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} IMU_PacketData_t;

/**
 * @brief IMU 初始化
 */
void App_IMU_Init(void);

/**
 * @brief IMU 姿态解算更新 (建议放入 10ms 定时器)
 */
void APP_IMU_Update(void);

/**
 * @brief 获取放大并转换后的整型数据，供串口封包发送使用
 * @param out_data 传出参数指针
 */
void APP_IMU_GetPacketData(IMU_PacketData_t *out_data);

#endif /*_APP_IMU_H*/
