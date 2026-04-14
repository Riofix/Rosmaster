#include "bsp_pwm.h"

/* -----------------------------------------------
 * PWM Channel Descriptor Table
 * CH1: PA6 (TIM3) -> 10kHz for BLDC
 * CH2: PB8 (TIM4) -> 50Hz for Servo 1
 * CH3: PB9 (TIM4) -> 50Hz for Servo 2
 * ----------------------------------------------- */
typedef struct {
    TIM_TypeDef*  tim;
    uint16_t      tim_channel;
    GPIO_TypeDef* gpio_port;
    uint16_t      gpio_pin;
    uint32_t      gpio_clk;
    uint32_t      tim_clk;
    uint16_t      prescaler;
    uint16_t      period;
    uint32_t      period_us;
} PWM_ChanDesc_t;

static const PWM_ChanDesc_t pwm_table[PWM_CH_MAX] = {
    /* CH1: PA6 -> TIM3_CH1 (BLDC Motor, 10kHz)
       72MHz/(7+1) = 9MHz. 9MHz/900 = 10kHz. Period=100us */
    { TIM3, TIM_Channel_1, GPIOA, GPIO_Pin_6, 
      RCC_APB2Periph_GPIOA, RCC_APB1Periph_TIM3, 7, 899, 100 },
      
    /* CH2: PB8 -> TIM4_CH3 (Servo 1, 50Hz)
       72MHz/(71+1) = 1MHz. 1MHz/20000 = 50Hz. Period=20000us */
    { TIM4, TIM_Channel_3, GPIOB, GPIO_Pin_8, 
      RCC_APB2Periph_GPIOB, RCC_APB1Periph_TIM4, 71, 19999, 20000 },
      
    /* CH3: PB9 -> TIM4_CH4 (Servo 2, 50Hz)
       72MHz/(71+1) = 1MHz. 1MHz/20000 = 50Hz. Period=20000us */
    { TIM4, TIM_Channel_4, GPIOB, GPIO_Pin_9, 
      RCC_APB2Periph_GPIOB, RCC_APB1Periph_TIM4, 71, 19999, 20000 },
};

/* Helper: Set compare value */
static void set_compare(TIM_TypeDef* tim, uint16_t channel, uint32_t val) {
    if      (channel == TIM_Channel_1) TIM_SetCompare1(tim, val);
    else if (channel == TIM_Channel_2) TIM_SetCompare2(tim, val);
    else if (channel == TIM_Channel_3) TIM_SetCompare3(tim, val);
    else if (channel == TIM_Channel_4) TIM_SetCompare4(tim, val);
}

/* Helper: OC init */
static void oc_init(TIM_TypeDef* tim, uint16_t channel, TIM_OCInitTypeDef* oc) {
    if      (channel == TIM_Channel_1) TIM_OC1Init(tim, oc);
    else if (channel == TIM_Channel_2) TIM_OC2Init(tim, oc);
    else if (channel == TIM_Channel_3) TIM_OC3Init(tim, oc);
    else if (channel == TIM_Channel_4) TIM_OC4Init(tim, oc);
}

/* Helper: Overwrite ARR/PSC for a timer */
static void tim_base_config(TIM_TypeDef* tim, uint16_t psc, uint16_t arr) {
    TIM_TimeBaseInitTypeDef TIM_IS;
    TIM_IS.TIM_Prescaler     = psc;
    TIM_IS.TIM_Period        = arr;
    TIM_IS.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_IS.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_IS);
}

void PWM_Init(void) {
    GPIO_InitTypeDef  GPIO_IS;
    TIM_OCInitTypeDef OC_IS;
    uint8_t i;

    /* 1. Enable clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* 2. Configure GPIO Pins */
    GPIO_IS.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    for (i = 0; i < PWM_CH_MAX; i++) {
        GPIO_IS.GPIO_Pin = pwm_table[i].gpio_pin;
        GPIO_Init(pwm_table[i].gpio_port, &GPIO_IS);
    }

    /* 3. Configure Timer Bases */
    // TIM3 for CH1 (10kHz)
    tim_base_config(TIM3, pwm_table[PWM_CH1].prescaler, pwm_table[PWM_CH1].period);
    // TIM4 for CH2/3 (50Hz)
    tim_base_config(TIM4, pwm_table[PWM_CH2].prescaler, pwm_table[PWM_CH2].period);

    /* 4. Configure Output Compare */
    OC_IS.TIM_OCMode      = TIM_OCMode_PWM1;
    OC_IS.TIM_OutputState = TIM_OutputState_Enable;
    OC_IS.TIM_OCPolarity  = TIM_OCPolarity_High;
    OC_IS.TIM_Pulse       = 0;
    
    for (i = 0; i < PWM_CH_MAX; i++) {
        oc_init(pwm_table[i].tim, pwm_table[i].tim_channel, &OC_IS);
        // Enable Preload
        if (pwm_table[i].tim_channel == TIM_Channel_1) TIM_OC1PreloadConfig(pwm_table[i].tim, TIM_OCPreload_Enable);
        else if (pwm_table[i].tim_channel == TIM_Channel_3) TIM_OC3PreloadConfig(pwm_table[i].tim, TIM_OCPreload_Enable);
        else if (pwm_table[i].tim_channel == TIM_Channel_4) TIM_OC4PreloadConfig(pwm_table[i].tim, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void PWM_SetPulse_us(PWM_Channel_t ch, uint16_t pulse_us) {
    if (ch >= PWM_CH_MAX) return;
    
    // Convert us to counts: val = (pulse_us / period_us) * (ARR + 1)
    // Avoid floating point: val = (uint32_t)pulse_us * (ARR + 1) / period_us
    uint32_t arr_plus_1 = (uint32_t)pwm_table[ch].period + 1;
    uint32_t val = (uint32_t)pulse_us * arr_plus_1 / pwm_table[ch].period_us;
    
    if (val > pwm_table[ch].period) val = pwm_table[ch].period;
    set_compare(pwm_table[ch].tim, pwm_table[ch].tim_channel, val);
}

void PWM_SetDuty(PWM_Channel_t ch, uint8_t duty) {
    if (ch >= PWM_CH_MAX) return;
    if (duty > 100) duty = 100;
    
    uint32_t val = (uint32_t)(pwm_table[ch].period + 1) * duty / 100;
    if (val > pwm_table[ch].period) val = pwm_table[ch].period;
    set_compare(pwm_table[ch].tim, pwm_table[ch].tim_channel, val);
}

void PWM_Servo_SetAngle(PWM_Channel_t ch, uint8_t angle) {
    // Only valid for 50Hz channels (CH2, CH3)
    if (ch < PWM_CH2 || ch > PWM_CH3) return;
    if (angle > 180) angle = 180;
    
    uint16_t pulse = PWM_SERVO_MIN_US
                   + (uint32_t)angle * (PWM_SERVO_MAX_US - PWM_SERVO_MIN_US) / 180;
    PWM_SetPulse_us(ch, pulse);
}

void PWM_Stop(PWM_Channel_t ch) {
    if (ch == PWM_CH_MAX) {
        uint8_t i;
        for (i = 0; i < PWM_CH_MAX; i++)
            set_compare(pwm_table[i].tim, pwm_table[i].tim_channel, 0);
    } else if (ch < PWM_CH_MAX) {
        set_compare(pwm_table[ch].tim, pwm_table[ch].tim_channel, 0);
    }
}
