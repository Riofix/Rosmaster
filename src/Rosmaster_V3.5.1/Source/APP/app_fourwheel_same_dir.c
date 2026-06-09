#include "app_fourwheel_same_dir.h"

#include "app_motion.h"
#include "app_pid.h"

#include "app.h"

#include "stdint.h"

#include "bsp_usart.h"
#include "bsp_motor.h"
#include "bsp_common.h"

static float speed_fb = 0;
static float speed_spin = 0;

static int speed_L1_setup = 0;
static int speed_L2_setup = 0;
static int speed_R1_setup = 0;
static int speed_R2_setup = 0;

static int g_offset_yaw = 0;
static uint16_t g_speed_setup = 0;

// X轴前进(前正后负：±1000)，Y轴不使用(0)，旋转(左正右负：±5000)
// 四轮同向：所有电机安装方向一致，不需要左右镜像反转
void FourWheelSameDir_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust)
{
    float robot_APB = Motion_Get_APB();
    speed_fb = V_x;
    (void)V_y;  // 四轮同向车不能横向移动，忽略Y轴
    speed_spin = (-V_z / 1000.0f) * robot_APB;
    if (V_x == 0 && V_z == 0)
    {
        Motion_Stop(STOP_BRAKE);
        return;
    }

    // 四轮同向差速：左侧两轮同速，右侧两轮同速
    speed_L1_setup = speed_fb + speed_spin;
    speed_L2_setup = speed_fb + speed_spin;
    speed_R1_setup = speed_fb - speed_spin;
    speed_R2_setup = speed_fb - speed_spin;

    if (speed_L1_setup > 1000) speed_L1_setup = 1000;
    if (speed_L1_setup < -1000) speed_L1_setup = -1000;
    if (speed_L2_setup > 1000) speed_L2_setup = 1000;
    if (speed_L2_setup < -1000) speed_L2_setup = -1000;
    if (speed_R1_setup > 1000) speed_R1_setup = 1000;
    if (speed_R1_setup < -1000) speed_R1_setup = -1000;
    if (speed_R2_setup > 1000) speed_R2_setup = 1000;
    if (speed_R2_setup < -1000) speed_R2_setup = -1000;

    Motion_Set_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
}


// 通过偏航角计算当前的偏差值，校准小车运动方向。
void FourWheelSameDir_Yaw_Calc(float yaw)
{
    float pid_result = PID_Yaw_Calc(yaw);
    g_offset_yaw = pid_result * g_speed_setup;

    int speed_L1 = speed_L1_setup - g_offset_yaw;
    int speed_L2 = speed_L2_setup - g_offset_yaw;
    int speed_R1 = speed_R1_setup + g_offset_yaw;
    int speed_R2 = speed_R2_setup + g_offset_yaw;

    if (speed_L1 > 1000) speed_L1 = 1000;
    if (speed_L1 < -1000) speed_L1 = -1000;
    if (speed_L2 > 1000) speed_L2 = 1000;
    if (speed_L2 < -1000) speed_L2 = -1000;
    if (speed_R1 > 1000) speed_R1 = 1000;
    if (speed_R1 < -1000) speed_R1 = -1000;
    if (speed_R2 > 1000) speed_R2 = 1000;
    if (speed_R2 < -1000) speed_R2 = -1000;

    Motion_Set_Speed(speed_L1, speed_L2, speed_R1, speed_R2);
}


// 控制四轮同向小车运动状态。
// 速度控制：speed=0~1000。
// 偏航角调节运动：adjust=1开启，=0不开启。
void FourWheelSameDir_State(uint8_t state, uint16_t speed, uint8_t adjust)
{
    g_speed_setup = speed;
    switch (state)
    {
    case MOTION_STOP:
        g_speed_setup = 0;
        Motion_Stop(speed == 0 ? STOP_FREE : STOP_BRAKE);
        break;
    case MOTION_RUN:
        Motion_Set_Yaw_Adjust(adjust);
        FourWheelSameDir_Ctrl(speed, 0, 0, adjust);
        break;
    case MOTION_BACK:
        Motion_Set_Yaw_Adjust(adjust);
        FourWheelSameDir_Ctrl(-speed, 0, 0, adjust);
        break;
    case MOTION_LEFT:
        Motion_Set_Yaw_Adjust(0);
        FourWheelSameDir_Ctrl(speed / 2, 0, speed * 2, adjust);
        break;
    case MOTION_RIGHT:
        Motion_Set_Yaw_Adjust(0);
        FourWheelSameDir_Ctrl(speed / 2, 0, -speed * 2, adjust);
        break;
    case MOTION_SPIN_LEFT:
        Motion_Set_Yaw_Adjust(0);
        FourWheelSameDir_Ctrl(0, 0, speed * 5, 0);
        break;
    case MOTION_SPIN_RIGHT:
        Motion_Set_Yaw_Adjust(0);
        FourWheelSameDir_Ctrl(0, 0, -speed * 5, 0);
        break;
    case MOTION_BRAKE:
        g_speed_setup = 0;
        Motion_Stop(STOP_BRAKE);
        break;
    default:
        g_speed_setup = 0;
        break;
    }
}
