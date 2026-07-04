/**
 * @file    app_action.c
 * @brief   两个独立动作函数: MoveTo (一次性) + Grab (内部状态机)
 *
 * @details
 *   App_Action_MoveTo  →  cmd_handle 收到 0x7A 时直接调用, 算路径发指令即返回
 *   App_Action_Grab    →  App_Tick 末尾周期调用, 内部状态机推进抓取序列
 *
 *   到位检测: 电机驱动 ACK 可能携带旧 move 的到位标志,
 *   因此必须"先见 flag bit1=0, 再见 bit1=1"才算真正到位。
 */

#include "app_action.h"
#include "app_motor.h"
#include "app_bldc.h"
#include "Emm_V5.h"
#include "protocol.h"
#include "cmd_handle.h"

/* ================================================================
 * 脉冲参数
 * ================================================================ */
#define SWEEP_PULSE 6472    /* 电机1 横扫 18cm */
#define DOWN10_PULSE 160000 /* 电机2 下降 10cm */
#define DOWN1_PULSE 16000   /* 电机2 下降 1cm */
#define UP13_PULSE 208000   /* 电机2 上升 13cm */

/* ================================================================
 * 速度参数
 * ================================================================ */
#define VEL_HORIZ 50 /* 电机1 横扫速度 */
#define VEL_MOVE 200 /* 电机1 点位移动速度 */
#define VEL_VERT 500 /* 电机2 垂直速度 */
#define ACC 100      /* 统一加速度 */

/* ================================================================
 * Grab 状态机内部枚举 (不暴露给外部)
 * ================================================================ */
typedef enum
{
    GRAB_IDLE = 0,   /* 空闲, 不在执行抓取 */
    GRAB_EMERGENCY,  /* 紧急停止 */
    GRAB_RESET,      /* 复位: 电机1/2 回原点 */
    GRAB_DOWN10,     /* 降 10cm */
    GRAB_BLDC_ON,    /* 开无刷 */
    GRAB_SWEEP_CCW,  /* 逆时针横扫 */
    GRAB_DOWN1_A,    /* 降 1cm (第1次) */
    GRAB_SWEEP_CW,   /* 顺时针横扫 */
    GRAB_DOWN1_B,    /* 降 1cm (第2次) */
    GRAB_SWEEP_CCW2, /* 逆时针横扫 (第2组) */
    GRAB_DOWN1_C,    /* 降 1cm (第3次) */
    GRAB_SWEEP_CW2,  /* 顺时针横扫 (第2组) */
    GRAB_BLDC_OFF,   /* 关无刷 */
    GRAB_UP13        /* 升 13cm 回位 */
} GrabStep_t;

/* Grab 静态状态 */
static GrabStep_t s_grab_step = GRAB_IDLE; /* 当前步骤 */
static uint8_t s_grab_flag_low = 0;        /* 到位清零检测 (bit0=电机1, bit1=电机2) */
static uint32_t s_grab_timeout = 0;        /* 超时计数 */
#define GRAB_TIMEOUT 30000                 /* 30 秒超时 (tick数) */

/* ================================================================
 * 原点偏移量: 上电后编码器归零, 但抓手实际在某一物理位上。
 * SetOrigin 记录 offset = 物理位脉冲 - 当前编码器值(0),
 * 之后 MoveTo 用 (编码器 + offset) 作为真实物理位置。
 * ================================================================ */
static uint8_t origin_pos_id = 1;        /* 上电后默认原点在 pos1 */
static uint32_t origin_pulse_offset = 0; /* 原点偏移量 */

void App_Action_SetOrigin(uint8_t pos_id)
{
    if (pos_id < 1 || pos_id > 8)
        return;
    origin_pos_id = pos_id;
    origin_pulse_offset = POS_PULSE(pos_id); /* 原点偏移量 */
}

/* ================================================================
 * App_Action_MoveTo
 *
 *   环轨点位移动, 一次性函数。
 *   cmd_handle 收到 0x7A 时直接调用, 计算路径 → 发指令 → 返回。
 *   内部使用 (编码器 + offset) 作为真实物理位置参与环形计算。
 * ================================================================ */
void App_Action_MoveTo(uint8_t pos_id, uint8_t clockwise)
{
    uint32_t target_pulse, cur_pulse, rel_pulse;
    uint32_t move_pulse; /* 需要移动脉冲数 */
    uint8_t dir;

    /* 参数保护 */
    if (pos_id < 1 || pos_id > 8)
        return;
    target_pulse = POS_PULSE(pos_id);

    /* 真实物理位置 = 编码器 + 原点偏移, 归一化到 [0, CIRCLE_PULSE) */
    {
        /* 编码器→脉冲: 编码器65536/圈, 脉冲3200/圈 */
        int32_t cur_pulse = (int32_t)((int64_t)g_motors[0].current_pos * 3200 / 65536);
        rel_pulse = origin_pulse_offset - cur_pulse; /* 归一化到原点 */
        if (rel_pulse >= CIRCLE_PULSE)
            rel_pulse = rel_pulse % CIRCLE_PULSE;
        else if (rel_pulse < 0)
            rel_pulse = (rel_pulse % CIRCLE_PULSE) + CIRCLE_PULSE;
    }

    /* 按方向计算环形路径上的相对脉冲 */
    if (target_pulse == rel_pulse)
    {
        move_pulse = 0; /* 已在目标位, 不走 */
        dir = 0;
    }
    else if (clockwise)
    {
        /* 顺时针 (反转 dir=1): 脉冲递增方向 */
        if (target_pulse > rel_pulse)
            move_pulse = target_pulse - rel_pulse;
        else
            move_pulse = (CIRCLE_PULSE - rel_pulse) + target_pulse;
        dir = 1;
    }
    else
    {
        /* 逆时针 (正转 dir=0): 脉冲递减方向 */
        if (target_pulse < rel_pulse)
            move_pulse = rel_pulse - target_pulse;
        else
            move_pulse = rel_pulse + (CIRCLE_PULSE - target_pulse);
        dir = 0;
    }

    /* 发相对位置指令, 不等到位, 直接返回 */
    Emm_V5_Pos_Control(1, dir, VEL_MOVE, ACC, move_pulse, 0, 0);
}

/*
    @brief  紧急停止
    @param  None
    @retval None
*/
void App_Action_EmergencyStop(void)
{
    s_grab_step = GRAB_EMERGENCY;
    App_Bldc_Stop();
    Emm_V5_Pos_Control(1, 0, 0, 0, 0, 0, 0);
    Emm_V5_Pos_Control(2, 0, 0, 0, 0, 0, 0);
}

/* ================================================================
 * App_Action_Grab (内部状态机)
 *
 *   调用方式: 每次 App_Tick 末尾调用一次本函数。
 *   空闲时立刻返回, 几乎零开销。
 *
 *   流程:
 *     GRAB_IDLE 被外部置为非空闲 → 发射当前步指令
 *     → 等待对应电机到位 (先清零后置位) → 推进到下一步
 *     → 最后一步完成后自动回到 GRAB_IDLE
 *
 *   怎么启动:
 *     cmd_handle 中直接把 s_grab_step 设为 GRAB_DOWN10 即可
 *     (或者提供一个 App_Action_GrabStart() 包装)
 * ================================================================ */
void App_Action_Grab(void)
{
    /* ---- 空闲状态: 什么也不做 ---- */
    if (s_grab_step == GRAB_IDLE)
        return;

    /* ---- 超时保护: 30 秒没完成自动回到空闲 ---- */
    s_grab_timeout++;
    if (s_grab_timeout > GRAB_TIMEOUT)
    {
        s_grab_step = GRAB_IDLE;
        App_Bldc_Stop();
        return;
    }

    if (s_grab_step == GRAB_EMERGENCY)
    {
        /* 紧急停止后, 需要手动复位才能继续抓取 */
        return;
    }

    //---- 复位状态: 电机1/2 回原点 ----
    if (s_grab_step == GRAB_RESET)
    {
        /* 复位: 电机1/2 回原点 */
        Emm_V5_Pos_Control(1, 0, VEL_MOVE, ACC, origin_pulse_offset, 1, 0);
        Emm_V5_Pos_Control(2, 0, VEL_VERT, ACC, 0, 1, 0);
        g_motors[0].flag &= ~0x02; /* 清到位 */
        g_motors[1].flag &= ~0x02;
        //---- 等待电机1/2 到位 ----
        if (g_motors[0].flag & 0x02 && g_motors[1].flag & 0x02)
        {
            s_grab_step = GRAB_IDLE;
            s_grab_flag_low = 0;
            s_grab_timeout = 0;
            //---- 复位后清零编码器位置 ----
            Emm_V5_Reset_CurPos_To_Zero(1);
            Emm_V5_Reset_CurPos_To_Zero(2);
        }
        return;
    }

    /* ---- 到位检测: 电机1 (横扫) / 电机2 (升降) ----
     *
     *   必须先见 flag bit1 清零(电机开始移动), 再见置位(到达新位置)。
     *   这是为了防止指令发出后 ACK 里的旧到位标志被误判。
     * ------------------------------------------------------------ */
    switch (s_grab_step)
    {
    /* 电机2 等待: 降/升 */
    case GRAB_DOWN10:
    case GRAB_DOWN1_A:
    case GRAB_DOWN1_B:
    case GRAB_DOWN1_C:
    case GRAB_UP13:
        if (!(g_motors[1].flag & 0x02))
            s_grab_flag_low |= 0x02; /* 见识低位 → 标记"已清零" */
        else if (s_grab_flag_low & 0x02)
            s_grab_step++; /* 见识置位 + 已清零 → 到位, 推进 */
        break;

    /* 电机1 等待: 横扫 */
    case GRAB_SWEEP_CCW:
    case GRAB_SWEEP_CW:
    case GRAB_SWEEP_CCW2:
    case GRAB_SWEEP_CW2:
        if (!(g_motors[0].flag & 0x02))
            s_grab_flag_low |= 0x01; /* 见识低位 → 标记"已清零" */
        else if (s_grab_flag_low & 0x01)
            s_grab_step++; /* 见识置位 + 已清零 → 到位, 推进 */
        break;

    /* 无刷启停: 不用等, 下一步立即执行 */
    case GRAB_BLDC_ON:
    case GRAB_BLDC_OFF:
        s_grab_step++;
        break;

    default:
        break;
    }

    /* ---- 检查推进后是否到了最后一步 (GRAB_UP13 之后) ---- */
    if (s_grab_step > GRAB_UP13)
    {
        uint8_t done = CMD_TX_ACTION_DONE;
        Protocol_PackAndSend(&done, 1); /* 通知上位机抓取完成 */
        s_grab_step = GRAB_IDLE;
        s_grab_flag_low = 0;
        s_grab_timeout = 0;
        return;
    }

    /* ---- 如果刚推进到新状态, 发射下一步指令 ---- */
    {
        /* 用局部静态变量记住上一次发射的状态, 避免同一状态重复发指令 */
        static GrabStep_t s_last_sent = GRAB_IDLE;

        if (s_grab_step != s_last_sent)
        {
            s_last_sent = s_grab_step;
            s_grab_flag_low = 0; /* 新状态重新开始到位检测 */
            s_grab_timeout = 0;  /* 重置超时 */

            switch (s_grab_step)
            {
            case GRAB_DOWN10:
                /* 电机2 反转(下降) 10cm */
                Emm_V5_Pos_Control(2, 1, VEL_VERT, ACC, DOWN10_PULSE, 0, 0);
                g_motors[1].flag &= ~0x02; /* 清到位, 确保清零检测能触发 */
                break;

            case GRAB_BLDC_ON:
                App_Bldc_Run(100);
                break;

            case GRAB_SWEEP_CCW:
                /* 电机1 正转(逆时针) 横扫 18cm */
                Emm_V5_Pos_Control(1, 0, VEL_HORIZ, ACC, SWEEP_PULSE, 0, 0);
                g_motors[0].flag &= ~0x02;
                break;

            case GRAB_DOWN1_A:
            case GRAB_DOWN1_B:
            case GRAB_DOWN1_C:
                /* 电机2 反转(下降) 1cm */
                Emm_V5_Pos_Control(2, 1, VEL_VERT, ACC, DOWN1_PULSE, 0, 0);
                g_motors[1].flag &= ~0x02;
                break;

            case GRAB_SWEEP_CW:
                /* 电机1 反转(顺时针) 横扫 18cm */
                Emm_V5_Pos_Control(1, 1, VEL_HORIZ, ACC, SWEEP_PULSE, 0, 0);
                g_motors[0].flag &= ~0x02;
                break;

            case GRAB_SWEEP_CCW2:
                Emm_V5_Pos_Control(1, 0, VEL_HORIZ, ACC, SWEEP_PULSE, 0, 0);
                g_motors[0].flag &= ~0x02;
                break;

            case GRAB_SWEEP_CW2:
                Emm_V5_Pos_Control(1, 1, VEL_HORIZ, ACC, SWEEP_PULSE, 0, 0);
                g_motors[0].flag &= ~0x02;
                break;

            case GRAB_BLDC_OFF:
                App_Bldc_Stop();
                break;

            case GRAB_UP13:
                /* 电机2 正转(上升) 13cm */
                Emm_V5_Pos_Control(2, 0, VEL_VERT, ACC, UP13_PULSE, 0, 0);
                g_motors[1].flag &= ~0x02;
                break;

            default:
                break;
            }
        }
    }
}

/* ================================================================
 * 包装函数: 供 cmd_handle 调用, 启动抓取
 * ================================================================ */
void App_Action_GrabStart(void)
{
    if (s_grab_step == GRAB_IDLE)
    {
        s_grab_step = GRAB_DOWN10;
        s_grab_flag_low = 0;
        s_grab_timeout = 0;
    }
}
