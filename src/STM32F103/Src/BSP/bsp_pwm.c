#include "bsp_pwm.h"

/* -----------------------------------------------
 * PWM Channel Descriptor Table
 * Each entry describes one PWM channel hardware.
 * To add a new channel: add enum entry + table row.
 * ----------------------------------------------- */
typedef struct {
    TIM_TypeDef*  tim;
    uint16_t      tim_channel;
    GPIO_TypeDef* gpio_port;
    uint16_t      gpio_pin;
    uint32_t      gpio_clk;
    uint32_t      tim_clk;
    uint8_t       is_apb2_tim;
    uint16_t      prescaler;
    uint32_t      period;
} PWM_ChanDesc_t;

static const PWM_ChanDesc_t pwm_table[PWM_CH_MAX] = {
    /* CH1: PA6 -> TIM3_CH1 (BLDC) */
    { TIM3, TIM_Channel_1, GPIOA, GPIO_Pin_6,
      RCC_APB2Periph_GPIOA, RCC_APB1Periph_TIM3, 0, 71, 19999 },
    /* CH2: PB0 -> TIM3_CH3 (Servo 1) */
    { TIM3, TIM_Channel_3, GPIOB, GPIO_Pin_0,
      RCC_APB2Periph_GPIOB, RCC_APB1Periph_TIM3, 0, 71, 19999 },
    /* CH3: PB1 -> TIM3_CH4 (Servo 2) */
    { TIM3, TIM_Channel_4, GPIOB, GPIO_Pin_1,
      RCC_APB2Periph_GPIOB, RCC_APB1Periph_TIM3, 0, 71, 19999 },
};

/* Set the compare register for a given timer + channel */
static void set_compare(TIM_TypeDef* tim, uint16_t channel, uint32_t val) {
    if      (channel == TIM_Channel_1) TIM_SetCompare1(tim, val);
    else if (channel == TIM_Channel_2) TIM_SetCompare2(tim, val);
    else if (channel == TIM_Channel_3) TIM_SetCompare3(tim, val);
    else if (channel == TIM_Channel_4) TIM_SetCompare4(tim, val);
}

/* OC init helper for any channel */
static void oc_init(TIM_TypeDef* tim, uint16_t channel,
                    TIM_OCInitTypeDef* oc) {
    if      (channel == TIM_Channel_1) TIM_OC1Init(tim, oc);
    else if (channel == TIM_Channel_2) TIM_OC2Init(tim, oc);
    else if (channel == TIM_Channel_3) TIM_OC3Init(tim, oc);
    else if (channel == TIM_Channel_4) TIM_OC4Init(tim, oc);
}

/* Preload helper */
static void oc_preload(TIM_TypeDef* tim, uint16_t channel) {
    if      (channel == TIM_Channel_1) TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
    else if (channel == TIM_Channel_2) TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    else if (channel == TIM_Channel_3) TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
    else if (channel == TIM_Channel_4) TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
}

void PWM_Init(void) {
    GPIO_InitTypeDef       GPIO_IS;
    TIM_TimeBaseInitTypeDef TIM_IS;
    TIM_OCInitTypeDef      OC_IS;
    uint8_t i;

    /* Enable all clocks first */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB
                           | RCC_APB2Periph_AFIO, ENABLE);

    /* Configure GPIO for all channels */
    GPIO_IS.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    for (i = 0; i < PWM_CH_MAX; i++) {
        GPIO_IS.GPIO_Pin = pwm_table[i].gpio_pin;
        GPIO_Init(pwm_table[i].gpio_port, &GPIO_IS);
    }

    /* Configure TIM3 time base (shared by all 3 channels) */
    TIM_IS.TIM_Prescaler     = pwm_table[0].prescaler;
    TIM_IS.TIM_Period        = pwm_table[0].period;
    TIM_IS.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_IS.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_IS);

    /* Configure output compare for each channel */
    OC_IS.TIM_OCMode      = TIM_OCMode_PWM1;
    OC_IS.TIM_OutputState = TIM_OutputState_Enable;
    OC_IS.TIM_OCPolarity  = TIM_OCPolarity_High;
    OC_IS.TIM_Pulse       = 0;
    for (i = 0; i < PWM_CH_MAX; i++) {
        oc_init(pwm_table[i].tim, pwm_table[i].tim_channel, &OC_IS);
        oc_preload(pwm_table[i].tim, pwm_table[i].tim_channel);
    }

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void PWM_SetPulse_us(PWM_Channel_t ch, uint16_t pulse_us) {
    if (ch >= PWM_CH_MAX) return;
    if (pulse_us >= PWM_PERIOD_US) pulse_us = PWM_PERIOD_US - 1;
    set_compare(pwm_table[ch].tim, pwm_table[ch].tim_channel, pulse_us);
}

void PWM_SetDuty(PWM_Channel_t ch, uint8_t duty) {
    if (ch >= PWM_CH_MAX) return;
    if (duty > 100) duty = 100;
    uint32_t pulse = (uint32_t)PWM_PERIOD_US * duty / 100;
    set_compare(pwm_table[ch].tim, pwm_table[ch].tim_channel, pulse);
}

void PWM_Servo_SetAngle(PWM_Channel_t ch, uint8_t angle) {
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
