#include "app_imu.h"

void App_IMU_Init(void)
{
    MPU_Init();
}

void APP_IMU_Update(void)
{
    if (MPU_ReadRawData(&g_mpu_raw) == 0)
    {
        // 假设外部调用频率为 10ms = 0.01s
        MPU_QuaternionUpdate(&g_mpu_raw, 0.01f);
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
