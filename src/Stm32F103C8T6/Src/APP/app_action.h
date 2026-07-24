#ifndef __APP_ACTION_H
#define __APP_ACTION_H

#include <stdint.h>

/* 点位脉冲表 (pos_id 1~8, 索引 0~7) */
// #define POS_PULSE(pos) ((uint32_t[]){0, 14382, 28764, 46741, 123325, 101393, 11146, 64000}[(pos) - 1])
//#define POS_PULSE(pos) ((uint32_t[]){0, 10667, 21334, 34667, 91468, 75201, 8267, 47467}[(pos) - 1])
#define POS_PULSE(pos) ((uint32_t[]){0, 8784, 17568, 27450, 70529, 58414, 6808, 62981}[(pos) - 1])

/* 环形轨道一圈脉冲数 */
// #define CIRCLE_PULSE 141303
#define CIRCLE_PULSE 104802

/**
 * @brief  电机1 环轨点位移动 (一次性, 不等到位)
 * @param  pos_id     目标位置号 1~8
 * @param  clockwise  1=顺时针(反转), 0=逆时针(正转)
 *
 * 内部: 读当前位置 → 归一化 → 按方向算环形路径距离 → 发相对位置指令 → 返回
 */
void App_Action_MoveTo(uint8_t pos_id, uint8_t clockwise);

/**
 * @brief  抓取动作状态机 (由 App_Tick 周期调用)
 *
 * 调用一次发射当前步指令, 到位后自动推进到下一步。
 * 没在执行时立即返回, CPU 开销为零。
 *
 * 抓取序列:
 *      降10cm →
 *      开无刷 →
 *      顺扫18cm →
 *      降1cm →
 *      逆扫18cm →
 *      ----取消这里的步骤-------若需要则把注释打开
 *      降1cm →
 *      逆扫18cm →
 *      降1cm →
 *      顺扫18cm →
 *      ----取消这里的步骤-------
 *      关无刷 →
 *      升12cm →
 *      完成
 */
void App_Action_Grab(void);                /* 内部状态机, App_Tick 末尾周期调用 */
void App_Action_GrabStart(void);           /* 启动抓取, cmd_handle 调用 */
void App_Action_SetOrigin(uint8_t pos_id); /* 校准原点: 记录编码器0对应的物理位 */
void App_Action_EmergencyStop(void);       /* 紧急停止: 立即停电机, 需要手动复位才能继续抓取 */
#endif                                     /* __APP_ACTION_H */
