#include "bsp_timer.h"

/**
 * @brief  初始化 TIM2 为自由运行微秒计数器
 *         TIM2 挂在 APB1, 时钟 72MHz
 *         预分频 72 → 1MHz 计数, 每 µs +1
 */
void DtTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    /* 开启 TIM2 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* 默认参数 */
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);

    /* 72MHz / 7200 = 10kHz → 每 100µs 计一次数 */
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;

    /* 16 位满量程 65535, 65535 × 100µs ≈ 6.55 秒溢出 */
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;

    /* 向上计数 */
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    /* 不重复, 不分频 */
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    /* 启动定时器 */
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief  读取当前微秒计数值
 */
uint32_t DtTimer_GetUs(void)
{
    /* TIM2 每 tick = 100µs, 转换为微秒 */
    return TIM_GetCounter(TIM2) * 100;
}
