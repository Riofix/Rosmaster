#include "bsp_tcs3472.h"
#include "bsp_iic.h"        // 底层 I2C 时序函数
#include "bsp_systick.h"    // 延时

/* ----------------------------- 底层读写封装 ----------------------------- */
/**
 * @brief 向 TCS3472 指定寄存器写入一个字节
 * @param reg  寄存器地址（低5位有效）
 * @param data 要写入的数据
 * @return 1成功，0失败
 */
static uint8_t TCS3472_WriteReg(uint8_t reg, uint8_t data) {
    IIC_Start();
    IIC_Send_Byte(TCS3472_ADDR_WRITE);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    // 发送命令字节：bit7=1，低5位为寄存器地址
    IIC_Send_Byte(TCS3472_CMD_BIT | reg);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    IIC_Send_Byte(data);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    IIC_Stop();
    return 1;
}

/**
 * @brief 从 TCS3472 指定寄存器读取一个字节
 * @param reg  寄存器地址
 * @param data 读取结果存放指针
 * @return 1成功，0失败
 */
static uint8_t TCS3472_ReadReg(uint8_t reg, uint8_t *data) {
    IIC_Start();
    IIC_Send_Byte(TCS3472_ADDR_WRITE);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    IIC_Send_Byte(TCS3472_CMD_BIT | reg);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    // 重复起始条件，转为读模式
    IIC_Start();
    IIC_Send_Byte(TCS3472_ADDR_READ);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    *data = IIC_Read_Byte(0);   // 最后一个字节发送 NACK
    IIC_Stop();
    return 1;
}

/**
 * @brief 连续读取多个寄存器（使用自动递增）
 * @param start_reg 起始寄存器地址
 * @param buf       数据缓冲区
 * @param len       读取字节数
 * @return 1成功，0失败
 */
static uint8_t TCS3472_ReadRegs(uint8_t start_reg, uint8_t *buf, uint8_t len) {
    IIC_Start();
    IIC_Send_Byte(TCS3472_ADDR_WRITE);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    // 命令字节带自动递增位，便于连续读取
    IIC_Send_Byte(TCS3472_CMD_BIT | TCS3472_CMD_AUTO_INC | start_reg);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    IIC_Start();
    IIC_Send_Byte(TCS3472_ADDR_READ);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    for (uint8_t i = 0; i < len; i++) {
        if (i == len - 1)
            buf[i] = IIC_Read_Byte(0);  // 最后一字节 NACK
        else
            buf[i] = IIC_Read_Byte(1);  // 中间字节 ACK
    }
    IIC_Stop();
    return 1;
}

/* ----------------------------- 功能函数 ----------------------------- */

/**
 * @brief 初始化 TCS3472，配置为默认参数（增益16x，积分时间24ms）
 * @return 
 */
void TCS3472_Init(void) {

    // 设置使能：先上电(PON)，等待2.4ms后再使能RGBC(AEN)
    TCS3472_WriteReg(TCS3472_ENABLE, TCS3472_EN_PON);
    Delay_ms(3);  // 等待稳定
    TCS3472_WriteReg(TCS3472_ENABLE, TCS3472_EN_PON | TCS3472_EN_AEN);

    // 设置积分时间：24ms（ATIME=0xF6）
    TCS3472_WriteReg(TCS3472_ATIME, TCS3472_ATIME_24MS);

    // 设置增益：16倍（常见室内光照）
    TCS3472_WriteReg(TCS3472_CONTROL, TCS3472_AGAIN_16X);
}

/**
 * @brief 读取一次 RGBC 数据（非阻塞版本）
 * @param data 数据指针
 * @return 无（若数据未就绪，则不会更新 data 指向的内容）
 */
void TCS3472_ReadRGBC(TCS3472_RGBC_Data *data) {
    uint8_t status;
    uint8_t buf[8];

    // 1. 读取状态寄存器
    if (!TCS3472_ReadReg(TCS3472_STATUS, &status)) {
        return; // I2C 读取失败直接返回
    }

    // 2. 核心改动：去掉 do-while，改用 if 判断
    // 如果 AVALID 位为 0，说明传感器还没完成本轮转换
    if (!(status & TCS3472_STATUS_AVALID)) {
        return; // 数据没准备好，直接“撤退”，不让 CPU 在这儿等
    }

    // 3. 只有数据有效时，才执行后续的连续读取
    if (!TCS3472_ReadRegs(TCS3472_CDATAL, buf, 8)) {
        return;  // 读取失败
    }

    // 4. 组装数据（逻辑与原代码一致）
    data->clear = buf[0] | (buf[1] << 8);
    data->red   = buf[2] | (buf[3] << 8);
    data->green = buf[4] | (buf[5] << 8);
    data->blue  = buf[6] | (buf[7] << 8);
}
