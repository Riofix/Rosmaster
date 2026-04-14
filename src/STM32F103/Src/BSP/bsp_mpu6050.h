#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include "stm32f10x.h"
#include <stdint.h>

/* =======================================================
 * MPU6050 I2C 设备地址 (AD0 接 GND -> 0x68, 接 VCC -> 0x69)
 * ======================================================= */
#define MPU_ADDR        0x68

/* =======================================================
 * MPU6050 寄存器地址宏定义
 * ======================================================= */
#define MPU_SELF_TESTX_REG      0x0D  // 自检寄存器 X
#define MPU_SELF_TESTY_REG      0x0E  // 自检寄存器 Y
#define MPU_SELF_TESTZ_REG      0x0F  // 自检寄存器 Z
#define MPU_SELF_TESTA_REG      0x10  // 自检寄存器 A

#define MPU_SAMPLE_RATE_REG     0x19  // 采样频率分频器
#define MPU_CFG_REG             0x1A  // 配置寄存器 (数字低通滤波器)
#define MPU_GYRO_CFG_REG        0x1B  // 陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG       0x1C  // 加速度计配置寄存器
#define MPU_MOTION_DET_REG      0x1F  // 运动检测阈值设置寄存器

#define MPU_FIFO_EN_REG         0x23  // FIFO 使能寄存器
#define MPU_I2CMST_CTRL_REG     0x24  // IIC 主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG    0x25  // IIC 从机0 器件地址寄存器
#define MPU_I2CSLV0_REG         0x26  // IIC 从机0 数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG    0x27  // IIC 从机0 控制寄存器
#define MPU_I2CSLV1_ADDR_REG    0x28
#define MPU_I2CSLV1_REG         0x29
#define MPU_I2CSLV1_CTRL_REG    0x2A
#define MPU_I2CSLV2_ADDR_REG    0x2B
#define MPU_I2CSLV2_REG         0x2C
#define MPU_I2CSLV2_CTRL_REG    0x2D
#define MPU_I2CSLV3_ADDR_REG    0x2E
#define MPU_I2CSLV3_REG         0x2F
#define MPU_I2CSLV3_CTRL_REG    0x30
#define MPU_I2CSLV4_ADDR_REG    0x31
#define MPU_I2CSLV4_REG         0x32
#define MPU_I2CSLV4_DO_REG      0x33
#define MPU_I2CSLV4_CTRL_REG    0x34
#define MPU_I2CSLV4_DI_REG      0x35

#define MPU_I2CMST_STA_REG      0x36  // IIC 主机状态寄存器
#define MPU_INTBP_CFG_REG       0x37  // 中断/旁路设置寄存器
#define MPU_INT_EN_REG          0x38  // 中断使能寄存器
#define MPU_INT_STA_REG         0x3A  // 中断状态寄存器

/* 输出数据寄存器 (大端序, 高字节在前) */
#define MPU_ACCEL_XOUTH_REG     0x3B  // 加速度 X 轴高8位
#define MPU_ACCEL_XOUTL_REG     0x3C  // 加速度 X 轴低8位
#define MPU_ACCEL_YOUTH_REG     0x3D  // 加速度 Y 轴高8位
#define MPU_ACCEL_YOUTL_REG     0x3E  // 加速度 Y 轴低8位
#define MPU_ACCEL_ZOUTH_REG     0x3F  // 加速度 Z 轴高8位
#define MPU_ACCEL_ZOUTL_REG     0x40  // 加速度 Z 轴低8位

#define MPU_TEMP_OUTH_REG       0x41  // 温度高8位
#define MPU_TEMP_OUTL_REG       0x42  // 温度低8位

#define MPU_GYRO_XOUTH_REG      0x43  // 陀螺仪 X 轴高8位
#define MPU_GYRO_XOUTL_REG      0x44  // 陀螺仪 X 轴低8位
#define MPU_GYRO_YOUTH_REG      0x45  // 陀螺仪 Y 轴高8位
#define MPU_GYRO_YOUTL_REG      0x46  // 陀螺仪 Y 轴低8位
#define MPU_GYRO_ZOUTH_REG      0x47  // 陀螺仪 Z 轴高8位
#define MPU_GYRO_ZOUTL_REG      0x48  // 陀螺仪 Z 轴低8位

#define MPU_I2CSLV0_DO_REG      0x63
#define MPU_I2CSLV1_DO_REG      0x64
#define MPU_I2CSLV2_DO_REG      0x65
#define MPU_I2CSLV3_DO_REG      0x66
#define MPU_I2CMST_DELAY_REG    0x67
#define MPU_SIGPATH_RST_REG     0x68  // 信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG    0x69  // 运动检测控制寄存器
#define MPU_USER_CTRL_REG       0x6A  // 用户控制寄存器
#define MPU_PWR_MGMT1_REG       0x6B  // 电源管理寄存器1
#define MPU_PWR_MGMT2_REG       0x6C  // 电源管理寄存器2
#define MPU_FIFO_CNTH_REG       0x72  // FIFO 计数高8位
#define MPU_FIFO_CNTL_REG       0x73  // FIFO 计数低8位
#define MPU_FIFO_RW_REG         0x74  // FIFO 读写寄存器
#define MPU_DEVICE_ID_REG       0x75  // 器件ID寄存器 (固定返回 0x68)

/* =======================================================
 * 互补滤波参数
 * ======================================================= */
#define MPU_COMP_ALPHA      0.98f   // 陀螺仪权重 (1-alpha = 加速计权重)
#define MPU_SAMPLE_DT       0.002f  // 采样间隔 dt (s), 与主循环 2ms 对齐
#define MPU_CALIB_SAMPLES   200     // 零偏校准采样数

/* =======================================================
 * 数据结构体定义
 * ======================================================= */

/** @brief MPU6050 六轴原始ADC数据 + 温度  */
typedef struct {
    int16_t ax;     // 加速度 X 轴 (LSB)
    int16_t ay;     // 加速度 Y 轴 (LSB)
    int16_t az;     // 加速度 Z 轴 (LSB)
    int16_t temp;   // 温度原始值 (LSB)
    int16_t gx;     // 陀螺仪 X 轴 (LSB)
    int16_t gy;     // 陀螺仪 Y 轴 (LSB)
    int16_t gz;     // 陀螺仪 Z 轴 (LSB)
} MPU_RawData_t;

/** @brief 互补滤波输出姿态角 (单位: 度 °) */
typedef struct {
    float roll;     // 横滚角 (绕 X 轴)
    float pitch;    // 俯仰角 (绕 Y 轴)
    float yaw;      // 偏航角 (绕 Z 轴, 仅陀螺积分, 会漂移)
} MPU_Attitude_t;

/* =======================================================
 * 全局姿态数据 (供 app_protocol 直接引用打包)
 * ======================================================= */
extern MPU_Attitude_t g_mpu_attitude;

/* =======================================================
 * 函数声明
 * ======================================================= */

/** @brief MPU6050 初始化, 返回 0 成功, 非0 失败 */
uint8_t MPU_Init(void);

/** @brief 读取全部原始数据到 raw 结构体, 返回 0 成功 */
uint8_t MPU_ReadRawData(MPU_RawData_t *raw);

/** @brief 以 raw 数据和 dt(秒) 更新互补滤波, 结果写入 g_mpu_attitude */
void MPU_ComplementaryFilter(MPU_RawData_t *raw, float dt);

/** @brief 静止状态下采集 MPU_CALIB_SAMPLES 次, 计算陀螺零偏并存储 */
void MPU_Calibrate(void);

/** @brief 获取当前经过校准的 Z 轴角速度 (单位: °/s) */
float MPU_Get_GyroZ_DPS(void);

/* 底层寄存器访问 (内部使用, 对外也可调用) */
uint8_t MPU_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU_ReadReg (uint8_t reg, uint8_t *data);
uint8_t MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* __BSP_MPU6050_H */
