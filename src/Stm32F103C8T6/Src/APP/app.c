#include "app.h"
#include "app_imu.h"
#include "app_rgb.h"
#include "app_bldc.h"
#include "app_servo.h"
#include "app_motor.h"
#include "cmd_handle.h"
#include "bsp_mpu6050.h"
#include "Emm_V5.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

App_Context_t g_app_context = {0};

/**
 * @brief 初始化所有应用逻辑
 */
void App_Init(void)
{
    // 1. 初始化
    App_Motor_Init();
    App_IMU_Init();
    App_Servo_Init();
    App_Bldc_Init();
    App_Rgb_Init();

    // 2. 初始化指令分发层 (含协议栈初始化)
    Cmd_Handle_Init();

    g_app_context.system_ready = 1;

    // 3. 自动上传默认开启
    g_app_context.mpu_stream = 1;
    g_app_context.stepmotor_stream = 1;
    g_app_context.rgb_serson_stream = 1;
    g_app_context.pwm_state_stream = 1;
}

/**
 * @brief 周期调度逻辑
 */
void App_Tick(void)
{
    static uint32_t tick_count = 0;
    tick_count++;

    // ---- 1.数据轮询处理 ----  10个主循环周期更新一次
    if (tick_count % 10 == 0)
    {
        APP_IMU_Update(); // 更新mpu原始数据

        // APP_RGB_Update(); // 更新RGB传感器数据

        // ---- 2. 电机状态轮询 (5x 频率周期) ----
        static uint8_t poll_motor_id = 1;
        App_Motor_RequestState(poll_motor_id);
        poll_motor_id = (poll_motor_id == 1) ? 2 : 1;
    }

    // ---- 2.自动上报 ---------
    if (g_app_context.mpu_stream == 1)
    {
        // 打包数据
        IMU_PacketData_t att_data;
        APP_IMU_GetPacketData(&att_data);
        uint8_t tx_buf[7];
        tx_buf[0] = CMD_TX_STREAM_MPU; // MPU数据上报指令

        // 3. 严谨的小端序拆解打包 (低字节在前，高字节在后)
        tx_buf[1] = (uint8_t)(att_data.roll & 0xFF);
        tx_buf[2] = (uint8_t)((att_data.roll >> 8) & 0xFF);

        tx_buf[3] = (uint8_t)(att_data.pitch & 0xFF);
        tx_buf[4] = (uint8_t)((att_data.pitch >> 8) & 0xFF);

        tx_buf[5] = (uint8_t)(att_data.yaw & 0xFF);
        tx_buf[6] = (uint8_t)((att_data.yaw >> 8) & 0xFF);

        Protocol_PackAndSend(tx_buf, sizeof(tx_buf)); // 发送数据
    }
    if (g_app_context.stepmotor_stream == 1)
    {
        // 先上报电机1的参数，再上报电机2的参数
        // 打包数据
        uint8_t tx_buf[24];
        tx_buf[0] = CMD_TX_STREAM_STEP;                         // 电机数据上报指令
        memcpy(tx_buf + 1, &g_motors[0], sizeof(MotorState_t)); // 电机1数据

        Protocol_PackAndSend(tx_buf, sizeof(tx_buf)); // 发送数据
        // 打包数据
        tx_buf[0] = CMD_TX_STREAM_STEP;                         // 电机数据上报指令
        memcpy(tx_buf + 1, &g_motors[1], sizeof(MotorState_t)); // 电机2数据

        Protocol_PackAndSend(tx_buf, sizeof(tx_buf)); // 发送数据
    }
    if (g_app_context.rgb_serson_stream == 1)
    {
    }
    if (g_app_context.pwm_state_stream == 1)
    {
        uint8_t tx_buf[5];
        tx_buf[0] = CMD_TX_STREAM_STATE;
        tx_buf[1] = g_servos[0].angle;
        tx_buf[2] = g_servos[1].angle;
        tx_buf[3] = g_bldc.duty; // 只发占空比

        Protocol_PackAndSend(tx_buf, 5); // 发送 5 字节
    }

    // ---- 3. OLED 显示逻辑执行 (取代 cmd_handle 中的直接显示) ----
    switch (g_app_context.oled_mode)
    {
    case 1:
        // 显示 MPU 数据
        OLED_ShowString(1, 1, "MPU Attitude");
        OLED_ShowString(2, 1, "R:");
        OLED_ShowSignedNum(2, 4, (int16_t)g_mpu_attitude.roll, 3);
        OLED_ShowString(3, 1, "P:");
        OLED_ShowSignedNum(3, 4, (int16_t)g_mpu_attitude.pitch, 3);
        OLED_ShowString(4, 1, "Y:");
        OLED_ShowSignedNum(4, 4, (int16_t)g_mpu_attitude.yaw, 3);
        break;

    case 2:
    {
        char buf[16];
        // 第1行：电机1 当前位置
        sprintf(buf, "M1 Pos:%6ld", g_motors[0].current_pos);
        OLED_ShowString(1, 1, buf);

        // 第2行：电机1 实时转速
        sprintf(buf, "M1 Vel:%4d", g_motors[0].velocity);
        OLED_ShowString(2, 1, buf);

        // 第3行：电机2 当前位置
        sprintf(buf, "M2 Pos:%6ld", g_motors[1].current_pos);
        OLED_ShowString(3, 1, buf);

        // 第4行：电机2 实时转速
        sprintf(buf, "M2 Vel:%4d", g_motors[1].velocity);
        OLED_ShowString(4, 1, buf);
        break;
    }

    default:
        // 可选：当 oled_mode 不是 1 或 2 时执行的默认动作
        // OLED_Clear();
        break;
    }
}
