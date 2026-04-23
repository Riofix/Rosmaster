#ifndef __APP_SERVO_H
#define __APP_SERVO_H

#include <stdint.h>
#include <stdbool.h>

#define SERVO_NUM 2

typedef struct
{
    uint8_t id;
    uint8_t angle;
} Servo_t;

extern Servo_t g_servos[SERVO_NUM];

void App_Servo_Init(void);
void App_Servo_SetAngle(uint8_t id, uint8_t angle);

#endif /* __APP_SERVO_H */
