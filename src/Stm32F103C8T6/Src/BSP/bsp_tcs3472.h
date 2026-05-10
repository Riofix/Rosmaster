#ifndef __BSP_TCS3472_H
#define __BSP_TCS3472_H

#include "stm32f10x.h"

// TCS3472 I2C 7位地址为 0x29，左移一位得 8位写地址 0x52
#define TCS3472_ADDR_WRITE    0x52      // (0x29 << 1) | 0
#define TCS3472_ADDR_READ     0x53      // (0x29 << 1) | 1

// 寄存器地址
#define TCS3472_ENABLE        0x00
#define TCS3472_ATIME         0x01
#define TCS3472_WTIME         0x03
#define TCS3472_AILTL         0x04
#define TCS3472_AILTH         0x05
#define TCS3472_AIHTL         0x06
#define TCS3472_AIHTH         0x07
#define TCS3472_PERS          0x0C
#define TCS3472_CONFIG        0x0D
#define TCS3472_CONTROL       0x0F
#define TCS3472_ID            0x12
#define TCS3472_STATUS        0x13
#define TCS3472_CDATAL        0x14
#define TCS3472_CDATAH        0x15
#define TCS3472_RDATAL        0x16
#define TCS3472_RDATAH        0x17
#define TCS3472_GDATAL        0x18
#define TCS3472_GDATAH        0x19
#define TCS3472_BDATAL        0x1A
#define TCS3472_BDATAH        0x1B

// 命令寄存器特殊位（写入寄存器地址前需组合的命令字节）
#define TCS3472_CMD_BIT       (1 << 7)       // 必须为1
#define TCS3472_CMD_AUTO_INC  (1 << 5)       // 自动递增，连续读取多个寄存器

// 使能寄存器位
#define TCS3472_EN_PON        (1 << 0)       // 电源使能
#define TCS3472_EN_AEN        (1 << 1)       // RGBC 使能
#define TCS3472_EN_WEN        (1 << 3)       // 等待使能（低功耗模式）
#define TCS3472_EN_AIEN       (1 << 4)       // 中断使能

// 控制寄存器增益设置 (AGAIN bit[1:0])
#define TCS3472_AGAIN_1X      (0x00)         // 1倍增益
#define TCS3472_AGAIN_4X      (0x01)         // 4倍增益
#define TCS3472_AGAIN_16X     (0x02)         // 16倍增益
#define TCS3472_AGAIN_60X     (0x03)         // 60倍增益

// 状态寄存器位
#define TCS3472_STATUS_AVALID (1 << 0)       // RGBC数据有效
#define TCS3472_STATUS_AINT   (1 << 4)       // 中断标志

// 常用积分时间预设值（ATIME寄存器值 = 256 - 积分时间/2.4ms）
#define TCS3472_ATIME_24MS    0xF6           // 24ms
#define TCS3472_ATIME_50MS    0xEB           // 50ms
#define TCS3472_ATIME_100MS   0xD6           // 100ms
#define TCS3472_ATIME_154MS   0xC0           // 154ms
#define TCS3472_ATIME_700MS   0x00           // 700ms


// RGBC 数据结构体
typedef struct {
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} TCS3472_RGBC_Data;

// 初始化函数
void TCS3472_Init(void);
// 读取 RGBC 原始数据（阻塞直到转换完成）
void TCS3472_ReadRGBC(TCS3472_RGBC_Data *data);

#endif
