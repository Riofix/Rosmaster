#ifndef __APP_TRACKER_H
#define __APP_TRACKER_H

#include <stdint.h>
#include <stdbool.h>

/* ================= 物理参数与硬件配置 ================= */
#define TRACKER_MOTOR_ADDR      1        /* 追踪里程主电机地址 */
#define TRACKER_WHEEL_D         24.0f    /* 24mm 轮径 */
#define TRACKER_MOTOR_RES       6400.0f  /* 6400 脉冲/圈 (0.9deg电机 400步 * 16细分) */

/* 脉冲当量: 6400 / (24 * PI) ≈ 84.8826 */
#define TRACKER_TICKS_PER_MM    84.8826f 

/* ================= 算法控制参数 ================= */
#define TRACKER_CRUISE_RPM      500      /* 默认巡航转速 (RPM) */
#define TRACKER_TURN_THRESHOLD  30.0f    /* 进入弯道的角速度阈值 (deg/s) */
#define TRACKER_KP              5.0f     /* 停靠 P 环比例系数 */
#define TRACKER_BRAKE_DIST_MM   200.0f   /* 提前 200mm 进入刹车模式 */
#define TRACKER_STALL_MS        500      /* 500ms 不动判定为堵转 */

/* ================= 轨道几何数据 (宏定义) ================= */
#define L1_STRAIGHT             1380.0f
#define L2_STRAIGHT             280.0f
#define R_CURVE_ARC             157.1f  /* 100mm 半径的 1/4 圆弧高度 */

/* 理论切点坐标 (基于累积距离) */
#define POINT_A_MM              (L1_STRAIGHT)
#define POINT_B_MM              (POINT_A_MM + R_CURVE_ARC + L2_STRAIGHT)
#define POINT_C_MM              (POINT_B_MM + R_CURVE_ARC + L1_STRAIGHT)
#define POINT_D_MM              (POINT_C_MM + R_CURVE_ARC + L2_STRAIGHT)
#define LAP_TOTAL_MM            (POINT_D_MM + R_CURVE_ARC)

/* ================= 函数接口 ================= */
void Tracker_Init(void);
void Tracker_SetTarget(float target_mm);
void Tracker_GetState(float *curr_pos, float *target, uint8_t *mode);

/** 
 * @brief  位置追踪核心更新函数
 * @param  abs_ticks  电机当前上报的绝对脉冲数
 * @param  gz_dps     MPU6050 当前 Z 轴角速度
 */
void Tracker_Update(int32_t abs_ticks, float gz_dps);
void Tracker_Reset(void);

#endif /* __APP_TRACKER_H */
