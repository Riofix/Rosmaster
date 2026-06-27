#include "app_imu.h"
#include "bsp_timer.h"

void App_IMU_Init(void)
{
    MPU_Init();
}

void APP_IMU_Update(void)
{
    static uint32_t last_us = 0;
    uint32_t now_us = DtTimer_GetUs();
    float dt = (float)(now_us - last_us) / 1000000.0f;
    last_us = now_us;

    /* 异常保护：首次调用或过长间隔 */
    if (dt <= 0.0f || dt > 0.5f)
        dt = 0.02f;

    if (MPU_ReadRawData(&g_mpu_raw) == 0)
    {
        MPU_QuaternionUpdate(&g_mpu_raw, dt);
    }
}

void APP_IMU_GetPacketData(IMU_PacketData_t *out_data)
{
    if (out_data != NULL)
    {
        // 从 bsp 层获取浮点型数据，放大 100 倍后强转为 int16_t
        out_data->roll = (int16_t)(g_mpu_attitude.roll * IMU_ANGLE_SCALE);
        out_data->pitch = (int16_t)(g_mpu_attitude.pitch * IMU_ANGLE_SCALE);
        out_data->yaw = (int16_t)(g_mpu_attitude.yaw * IMU_ANGLE_SCALE);
    }
}
