#include "app_tracker.h"
#include "Emm_V5.h"
#include <math.h>
#include <stdlib.h>

/* ================= 静态变量 ================= */
static float s_pos_mm = 0.0f;
static float s_target_mm = 0.0f;
static int32_t s_last_ticks = 0;
static float s_offset_mm = 0.0f; /* 单片机逻辑位姿相对于电机物理位置的偏移 */

static bool s_is_turning = false;
static bool s_last_turning = false;
static uint8_t s_mode = 0; /* 0: IDLE, 1: RUNNING, 2: BRAKING */

static int32_t s_last_sent_rpm = 0;

/* 理论标定点数组 */
static const float s_calib_points[] = {POINT_A_MM, POINT_B_MM, POINT_C_MM, POINT_D_MM};
#define CALIB_COUNT (sizeof(s_calib_points)/sizeof(float))

/* ================= 内部辅助函数 ================= */

/** @brief 发送速度指令 (包含重复指令过滤) */
static void Send_Vel_Cmd(int16_t rpm) {
    if (abs(rpm - s_last_sent_rpm) < 10 && rpm != 0) return; /* 差异过小不发送 */
    
    uint8_t dir = (rpm >= 0) ? 0 : 1;
    uint16_t vel = abs(rpm);
    Emm_V5_Vel_Control(TRACKER_MOTOR_ADDR, dir, vel, 20, false);
    s_last_sent_rpm = rpm;
}

/* ================= 接口函数实现 ================= */

void Tracker_Init(void) {
    s_pos_mm = 0.0f;
    s_target_mm = 0.0f;
    s_offset_mm = 0.0f;
    s_mode = 0;
    s_last_sent_rpm = 0;
}

void Tracker_SetTarget(float target_mm) {
    s_target_mm = target_mm;
    s_mode = 1;
    /* Kickstart: 立即下发第一条速度指令，不等待反馈 */
    Send_Vel_Cmd(TRACKER_CRUISE_RPM);
}

void Tracker_GetState(float *curr_pos, float *target, uint8_t *mode) {
    *curr_pos = s_pos_mm;
    *target = s_target_mm;
    *mode = s_mode;
}

void Tracker_Update(int32_t abs_ticks, float gz_dps) {
    /* 1. 物理位置解析 */
    /* 将电机的绝对脉冲转换为单片机的逻辑毫米 */
    s_pos_mm = (float)abs_ticks / TRACKER_TICKS_PER_MM + s_offset_mm;

    /* 2. 转弯特征检测与几何校准 */
    s_is_turning = (fabsf(gz_dps) > TRACKER_TURN_THRESHOLD);
    
    if (s_is_turning && !s_last_turning) {
        /* 检测到入弯瞬间 -> 寻找最近的理论标定点 */
        float min_err = 9999.0f;
        int target_idx = -1;
        float current_loop_pos = fmodf(s_pos_mm, LAP_TOTAL_MM);

        for (int i = 0; i < CALIB_COUNT; i++) {
            float err = fabsf(current_loop_pos - s_calib_points[i]);
            if (err < min_err && err < 150.0f) { /* 只校准 150mm 以内的点，防止误触发 */
                min_err = err;
                target_idx = i;
            }
        }

        if (target_idx != -1) {
            /* 执行逻辑位姿对齐 (方案 A: 逻辑同步) */
            float lap_base = floorf(s_pos_mm / LAP_TOTAL_MM) * LAP_TOTAL_MM;
            float ideal_pos = lap_base + s_calib_points[target_idx];
            
            /* 更新偏移量，使得下一步计算出的 s_pos_mm 等于 ideal_pos */
            s_offset_mm += (ideal_pos - s_pos_mm);
            s_pos_mm = ideal_pos;
        }
    }
    s_last_turning = s_is_turning;

    /* 3. 堵转检测与指令丢失补发 (Resilience Logic) */
    if (s_mode == 1) {
        static uint32_t stick_cnt = 0;
        if (abs_ticks == s_last_ticks) {
            if (++stick_cnt > 10) { /* 约 500ms 没动 */
                /* 可能是第一次指令丢了，或者是真的堵转了 */
                Emm_V5_Reset_Clog_Pro(TRACKER_MOTOR_ADDR); 
                Send_Vel_Cmd(TRACKER_CRUISE_RPM); /* 补发关键指令 */
                stick_cnt = 0;
            }
        } else {
            stick_cnt = 0;
        }
    }
    s_last_ticks = abs_ticks;

    /* 4. 自动控制决策机 (Autonomous Controller) */
    if (s_mode == 1) { /* RUNNING */
        float dist_go = s_target_mm - s_pos_mm;
        
        if (dist_go <= 0) {
            Emm_V5_Stop_Now(TRACKER_MOTOR_ADDR, false);
            s_mode = 0;
            s_last_sent_rpm = 0;
        } 
        else if (dist_go < TRACKER_BRAKE_DIST_MM) {
            /* 进入最后刹车区 -> 切换到位置模式实现精准停靠 */
            uint32_t remaining_ticks = (uint32_t)(dist_go * TRACKER_TICKS_PER_MM);
            /* 使用相对位置模式 raF = false */
            Emm_V5_Pos_Control(TRACKER_MOTOR_ADDR, 0, TRACKER_CRUISE_RPM/2, 50, remaining_ticks, false, false);
            s_mode = 2; /* 进入 BRAKING 锁定状态 */
        }
        else {
            /* 正常巡航 */
            Send_Vel_Cmd(TRACKER_CRUISE_RPM);
        }
    }
    else if (s_mode == 2) {
        /* BRAKING 模式下，单片机静默，等待电机硬件完成最后一段路程 */
        float dist_go = s_target_mm - s_pos_mm;
        if (dist_go < 2.0f) { /* 接近 2mm 认为停止 */
            s_mode = 0;
            s_last_sent_rpm = 0;
        }
    }
}

void Tracker_Reset(void) {
    s_pos_mm = 0;
    s_offset_mm = 0;
    s_target_mm = 0;
    s_mode = 0;
    s_last_ticks = 0;
    s_last_sent_rpm = 0;
    Emm_V5_Reset_CurPos_To_Zero(TRACKER_MOTOR_ADDR);
}
