#ifndef __APP_FOURWHEEL_SAME_DIR_H__
#define __APP_FOURWHEEL_SAME_DIR_H__

#include "stdint.h"

// 四轮同向车底盘电机间距之和的一半，单位mm（左轮与右轮之间的轴距/2）
#define FWSAME_APB                  (164.555f)

// 四轮同向车轮子转一整圈的位移，单位为mm
#define FWSAME_CIRCLE_MM            (215.2f)


void FourWheelSameDir_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust);
void FourWheelSameDir_State(uint8_t state, uint16_t speed, uint8_t adjust);
void FourWheelSameDir_Yaw_Calc(float yaw);

#endif /* __APP_FOURWHEEL_SAME_DIR_H__ */
