#include "app_bldc.h"
#include "bsp_pwm.h"

Bldc_t g_bldc;

void App_Bldc_Init(void)
{
    PWM_Bldc_Init();
}
void App_Bldc_Run(uint8_t duty)
{
    g_bldc.duty = duty;
    PWM_Bldc_SetDuty(g_bldc.duty);
}
void App_Bldc_Stop(void)
{
    g_bldc.duty = 0;
    PWM_Bldc_SetDuty(g_bldc.duty);
}
