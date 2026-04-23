#ifndef _PROTOCOL_EMM_H
#define _PROTOCOL_EMM_H

#include "stm32f10x.h" // Device header

typedef struct
{
    uint8_t addr;
    uint8_t func;
    uint8_t data[32];
    uint16_t len;
} Emm_Feedback_t;

typedef void (*Emm_Callback_t)(Emm_Feedback_t *msg);
void Protocol_Emm_Init(Emm_Callback_t callback);
void Protocol_Emm_Process(void);

#endif
