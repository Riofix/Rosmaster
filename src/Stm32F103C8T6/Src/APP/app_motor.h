#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol_emm.h"

/**
 * @brief 电机全量状态模型 (根据0x43返回值精简与结构化)
 */
typedef __packed struct
{
    uint8_t addr; // 电机地址
    /* ------ 物理遥测数据 ------ */
    uint16_t voltage_mv;   // 母线电压 (mV)
    int16_t phase_current; // 实时相电流 (mA)
    uint16_t encoder_val;  // 编码器线性化值
    int32_t target_pos;    // 目标位置
    int16_t velocity;      // 实时转速 (RPM)
    int32_t current_pos;   // 当前位置
    int32_t pos_error;     // 位置误差
    uint8_t org;           // 回零状态
    uint8_t flag;          // 电机状态标志

    // /* ------ 电机状态标志 (S_FLAG) ------ */
    // bool is_enabled;       // 1=电机处于使能状态
    // bool is_in_position;   // 1=电机已经到位
    // bool is_stalled;       // 0=没触发堵转, 1=触发堵转
    // bool stall_protection; // 0=没触发堵转保护, 1=触发堵转保护

    // /* ------ 回零与就绪标志 (S_ORG) ------ */
    // bool enc_ready;   // 1=编码器就绪
    // bool calib_ready; // 1=校准表就绪
    // bool is_homing;   // 0=当前没有回零
    // bool home_failed; // 0=没有回零失败
} MotorState_t;
/* =======================================================
 * 全局电机状态
 * ======================================================= */
extern MotorState_t g_motors[2]; // 电机状态数组

/**
 * @brief 初始化电机应用层
 */
void App_Motor_Init(void);

/**
 * @brief 获取单个电机的状态
 * @param idx 电机索引 (0 或 1)
 */
void App_Motor_GetState(uint8_t idx, MotorState_t *state);

/**
 * @brief 获取电机状态数组
 * @param idx 电机索引 (0 或 1)
 * @param state 电机状态数组
 */
int32_t App_Motor_GetParam(uint8_t idx, uint8_t param);

/**
 * @brief 内部使用的反馈回调函数 (供 protocol_emm 调用)
 */
void App_Motor_UpdateCallback(Emm_Feedback_t *msg);

/**
 * @brief 内部封装的轮询指令发送，发送 0x43全量查询
 * @param addr 目标电机地址
 */
void App_Motor_RequestState(uint8_t addr);

#endif
