#ifndef _APP_BLDC_H_
#define _APP_BLDC_H_

#include "stm32f10x.h"

typedef struct
{
    uint8_t duty;
} Bldc_t;

extern Bldc_t g_bldc;

void App_Bldc_Init(void);
void App_Bldc_Run(uint8_t duty);
void App_Bldc_Stop(void);

#endif
