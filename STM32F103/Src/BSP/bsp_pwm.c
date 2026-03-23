#include "bsp_pwm.h"

/**************************************************************************
函数功能：PWM初始化（TIM3通道1，PA6）
入口参数：无
返回  值：无
说明：20ms周期，50Hz
**************************************************************************/
void PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    
    // 配置PA6为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 定时器配置：72MHz/72=1MHz，计数频率1MHz
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_Period = 19999;      // 20ms周期
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // PWM配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;              // 初始占空比0
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    // 使能定时器
    TIM_Cmd(TIM3, ENABLE);
}

/**************************************************************************
函数功能：设置占空比
入口参数：duty - 占空比 0-100
返回  值：无
说明：0对应0%，100对应100%
**************************************************************************/
void PWM_SetDuty(uint8_t duty)
{
    uint16_t compare;
    
    // 限制范围
    if(duty > 100) duty = 100;
    
    // 计算比较值：占空比 = compare / 20000
    // compare = 20000 * duty / 100
    compare = (uint32_t)20000 * duty / 100;
    
    // 设置比较值
    TIM_SetCompare1(TIM3, compare);
}

