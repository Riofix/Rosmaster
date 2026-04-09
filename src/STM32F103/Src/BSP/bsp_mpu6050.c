/**
 * @file    bsp_mpu6050.c
 * @brief   MPU6050 驱动 + 互补滤波姿态解算
 * @details 通过 bsp_iic 软件 I2C 与 MPU6050 通信
 *          提供原始数据读取、陀螺零偏校准及 Roll/Pitch/Yaw 互补滤波输出
 *
 * @note    Yaw 轴仅靠陀螺积分, 无磁力计时存在长期漂移, 短时应用可接受
 */

#include "bsp_mpu6050.h"
#include "bsp_iic.h"
#include "bsp_systick.h"
#include <math.h>

/* =======================================================
 * 全局姿态数据 (由 extern 导出供上层使用)
 * ======================================================= */
MPU_Attitude_t g_mpu_attitude = {0.0f, 0.0f, 0.0f};

/* =======================================================
 * 内部静态变量
 * ======================================================= */
/* 陀螺仪零偏 (单位: 与原始 LSB 相同, 校准后减去) */
static float s_gyro_offset[3] = {0.0f, 0.0f, 0.0f};

/* 加速度计灵敏度: ±2g -> 16384 LSB/g */
#define ACCEL_SCALE     16384.0f
/* 陀螺仪灵敏度: ±500°/s -> 65.5 LSB/(°/s) */
#define GYRO_SCALE      65.5f

/* =======================================================
 * 底层寄存器读写封装
 * ======================================================= */

uint8_t MPU_WriteReg(uint8_t reg, uint8_t data) {
    return IICwriteByte(MPU_ADDR, reg, data);
}

uint8_t MPU_ReadReg(uint8_t reg, uint8_t *data) {
    return IICreadByte(MPU_ADDR, reg, data);
}

uint8_t MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    return IICreadBytes(MPU_ADDR, reg, len, buf);
}

/* =======================================================
 * MPU6050 初始化
 * ======================================================= */

/**
 * @brief  MPU6050 初始化
 * @retval 0: 成功, 1: WHO_AM_I 验证失败
 */
uint8_t MPU_Init(void) {
    uint8_t device_id = 0;

    /* 先验证器件 ID */
    MPU_ReadReg(MPU_DEVICE_ID_REG, &device_id);
    if (device_id != 0x68) {
        return 1; // 器件不存在或 I2C 通信失败
    }

    /* 复位器件, 然后退出睡眠模式 */
    MPU_WriteReg(MPU_PWR_MGMT1_REG, 0x80); // 软件复位
    Delay_ms(100);
    MPU_WriteReg(MPU_PWR_MGMT1_REG, 0x00); // 退出睡眠, 使用内部8MHz振荡器

    /* 采样率 = 陀螺输出率 / (1 + SMPLRT_DIV)
     * 陀螺输出率 1kHz (DLPF 开启时), SMPLRT_DIV=1 -> 500Hz */
    MPU_WriteReg(MPU_SAMPLE_RATE_REG, 0x01);

    /* 数字低通滤波器: DLPF_CFG=3 -> 带宽44Hz, 延迟4.9ms */
    MPU_WriteReg(MPU_CFG_REG, 0x03);

    /* 陀螺仪量程: ±500°/s (FS_SEL=1) */
    MPU_WriteReg(MPU_GYRO_CFG_REG, 0x08);

    /* 加速度计量程: ±2g (AFS_SEL=0) */
    MPU_WriteReg(MPU_ACCEL_CFG_REG, 0x00);

    /* 禁用 FIFO, 禁用中断 */
    MPU_WriteReg(MPU_FIFO_EN_REG, 0x00);
    MPU_WriteReg(MPU_INT_EN_REG,  0x00);

    return 0;
}

/* =======================================================
 * 原始数据读取
 * ======================================================= */

/**
 * @brief  读取 MPU6050 六轴 + 温度原始 ADC 数据
 * @param  raw  输出结构体指针
 * @retval 0: 成功, 1: I2C 读取失败
 */
uint8_t MPU_ReadRawData(MPU_RawData_t *raw) {
    uint8_t buf[14];

    /* 从 0x3B 连续读 14 字节: AX(2) AY(2) AZ(2) TEMP(2) GX(2) GY(2) GZ(2) */
    if (MPU_ReadRegs(MPU_ACCEL_XOUTH_REG, buf, 14) != 14) {
        return 1;
    }

    /* 大端拼接 */
    raw->ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->az   = (int16_t)((buf[4]  << 8) | buf[5]);
    raw->temp = (int16_t)((buf[6]  << 8) | buf[7]);
    raw->gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gy   = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gz   = (int16_t)((buf[12] << 8) | buf[13]);

    return 0;
}

/* =======================================================
 * 陀螺仪零偏校准
 * ======================================================= */

/**
 * @brief  静止放置板子, 采集 MPU_CALIB_SAMPLES 次陀螺数据计算零偏
 *         校准期间约需 MPU_CALIB_SAMPLES × dt = 0.4s
 */
void MPU_Calibrate(void) {
    MPU_RawData_t raw;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    int i;

    for (i = 0; i < MPU_CALIB_SAMPLES; i++) {
        MPU_ReadRawData(&raw);
        sum[0] += (float)raw.gx;
        sum[1] += (float)raw.gy;
        sum[2] += (float)raw.gz;
        Delay_ms(2); // 与主循环节奏一致
    }

    s_gyro_offset[0] = sum[0] / MPU_CALIB_SAMPLES;
    s_gyro_offset[1] = sum[1] / MPU_CALIB_SAMPLES;
    s_gyro_offset[2] = sum[2] / MPU_CALIB_SAMPLES;
}

/* =======================================================
 * 互补滤波姿态解算
 * ======================================================= */

/**
 * @brief  互补滤波更新 Roll / Pitch / Yaw
 *
 *  原理:
 *    1. 加速度计 -> 计算 Roll_acc, Pitch_acc (静态准确, 动态噪声大)
 *    2. 陀螺仪   -> 角速度积分更新角度 (短期准确, 长期漂移)
 *    3. 互补融合: angle = α*(angle + gyro_rate*dt) + (1-α)*accel_angle
 *    4. Yaw 无加速计修正, 仅靠陀螺积分 (存在漂移)
 *
 * @param  raw  当次读取的原始数据
 * @param  dt   调用间隔 (秒), 应与主循环一致 (建议 MPU_SAMPLE_DT = 0.002f)
 */
void MPU_ComplementaryFilter(MPU_RawData_t *raw, float dt) {
    /* ---- 1. 转换为物理量 ---- */
    float ax = (float)raw->ax / ACCEL_SCALE;  // 单位: g
    float ay = (float)raw->ay / ACCEL_SCALE;
    float az = (float)raw->az / ACCEL_SCALE;

    /* 减去零偏后转换为 °/s */
    float gx_dps = ((float)raw->gx - s_gyro_offset[0]) / GYRO_SCALE;
    float gy_dps = ((float)raw->gy - s_gyro_offset[1]) / GYRO_SCALE;
    float gz_dps = ((float)raw->gz - s_gyro_offset[2]) / GYRO_SCALE;

    /* ---- 2. 加速度计解算静态 Roll / Pitch ---- */
    /* Roll:  绕 X 轴倾斜角 */
    float roll_acc  = atan2f(ay, az) * (180.0f / 3.14159265f);
    /* Pitch: 绕 Y 轴倾斜角 */
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / 3.14159265f);

    /* ---- 3. 互补滤波融合 ---- */
    /* Roll  = α*(roll  + gx*dt) + (1-α)*roll_acc  */
    g_mpu_attitude.roll  = MPU_COMP_ALPHA * (g_mpu_attitude.roll  + gx_dps * dt)
                         + (1.0f - MPU_COMP_ALPHA) * roll_acc;

    /* Pitch = α*(pitch + gy*dt) + (1-α)*pitch_acc */
    g_mpu_attitude.pitch = MPU_COMP_ALPHA * (g_mpu_attitude.pitch + gy_dps * dt)
                         + (1.0f - MPU_COMP_ALPHA) * pitch_acc;

    /* Yaw: 仅靠陀螺积分 (无磁力计, 长期漂移) */
    g_mpu_attitude.yaw  += gz_dps * dt;

    /* Yaw 限制在 [-180, 180] */
    if (g_mpu_attitude.yaw >  180.0f) g_mpu_attitude.yaw -= 360.0f;
    if (g_mpu_attitude.yaw < -180.0f) g_mpu_attitude.yaw += 360.0f;
}
