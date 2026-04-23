#include "bsp_pwm.h"

/* -----------------------------------------------
 * PWM Channel Descriptor Table
 * CH1: PA6 (TIM3) -> 10kHz for BLDC
 * CH3: PB8 (TIM4) -> 50Hz for Servo 1
 * CH4: PB9 (TIM4) -> 50Hz for Servo 2
 * ----------------------------------------------- */

void PWM_Bldc_Init(void)

{
    /* 1. Enable clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* 2. Configure GPIOs */
    /* PA6 (TIM3) */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 3. Configure PWM Timers */
    // TIM3 10KHz
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数器模式，选择向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;                 // 计数周期，即ARR的值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;               // 预分频器，即PSC的值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            // 重复计数器，高级定时器才会用到
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             // 将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元

    /* 4. Configure PWM Channels 输出比较初始化*/
    TIM_OCInitTypeDef TIM_OCInitStructure;                        // 定义结构体变量
    TIM_OCStructInit(&TIM_OCInitStructure);                       // 结构体初始化，若结构体没有完整赋值
                                                                  // 则最好执行此函数，给结构体所有成员都赋一个默认值
                                                                  // 避免结构体初值不确定的问题
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // 输出比较模式，选择PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     // 输出极性，选择为高，若选择极性为低，则输出高低电平取反
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                            // 初始的CCR值
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);                      // 将结构体变量交给TIM_OC1Init，配置TIM3的输出比较通道1
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);             // 使能TIM3的预装载寄存器

    /* 5. Start Timers */
    TIM_Cmd(TIM3, ENABLE);
}

void PWM_Servo_Init(void)
{
    /* 1. Enable clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 2. Configure GPIOs */
    /* PB8 (TIM4) */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* PB9 (TIM4) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 3. Configure PWM Timers */
    // TIM4 CH3 CH4 50Hz
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数器模式，选择向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;               // 计数周期，即ARR的值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;               // 预分频器，即PSC的值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            // 重复计数器，高级定时器才会用到
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);             // 将结构体变量交给TIM_TimeBaseInit，配置TIM4的时基单元

    /* 4. Configure PWM Channels 输出比较初始化*/
    TIM_OCInitTypeDef TIM_OCInitStructure;                        // 定义结构体变量
    TIM_OCStructInit(&TIM_OCInitStructure);                       // 结构体初始化，若结构体没有完整赋值
                                                                  // 则最好执行此函数，给结构体所有成员都赋一个默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // 输出比较模式，选择PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     // 输出极性，选择为高，若选择极性为低，则输出高低电平取反
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                            // 初始的CCR值
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);                      // 将结构体变量交给TIM_OC1Init，配置TIM4的输出比较通道3
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);                      // 将结构体变量交给TIM_OC1Init，配置TIM4的输出比较通道4
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);             // 使能TIM4的预装载寄存器
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);             // 使能TIM4的预装载寄存器

    /* 5. Start Timers */
    TIM_Cmd(TIM4, ENABLE);
}

void PWM_Servo_SetAngle(uint8_t channel, uint8_t angle)
{
    uint16_t pulse = Angle_To_Pulse((float)angle);

    if (channel == 1)
    {
        TIM_SetCompare3(TIM4, pulse); // PB8
    }
    else if (channel == 2)
    {
        TIM_SetCompare4(TIM4, pulse); // PB9
    }
}

static uint16_t Angle_To_Pulse(float angle)
{
    return (uint16_t)(500.0f + (angle / 180.0f) * 2000.0f + 0.5f);
}

void PWM_Bldc_SetDuty(uint8_t duty)
{
    uint16_t ccr;

    if (duty > 100)
        duty = 100; // 限幅保护

    // 将 0~100 映射到 0~ARR (ARR = 99)
    ccr = (uint16_t)((uint32_t)duty * 99 / 100);

    TIM_SetCompare1(TIM3, ccr); // TIM3_CH1 PA6
}

void PWM_Bldc_Stop(void)
{
    TIM_SetCompare1(TIM3, 0); // TIM3_CH1 PB6
}
