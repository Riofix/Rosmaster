#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "stm32f10x.h"

/* -------------------------------------------------------
 * IIC 物理层引脚定义 (软件模拟 I2C, GPIOB)
 * SCL -> PB6   SDA -> PB7
 * ------------------------------------------------------- */
#define IIC_CLK         RCC_APB2Periph_GPIOB
#define IIC_Port        GPIOB
#define IIC_Pin_SCL     GPIO_Pin_6
#define IIC_Pin_SDA     GPIO_Pin_7

/* -------------------------------------------------------
 * 引脚电平操作宏
 * ------------------------------------------------------- */
#define IIC_SCL_HIGH()  GPIO_SetBits(IIC_Port, IIC_Pin_SCL)
#define IIC_SCL_LOW()   GPIO_ResetBits(IIC_Port, IIC_Pin_SCL)
#define IIC_SDA_HIGH()  GPIO_SetBits(IIC_Port, IIC_Pin_SDA)
#define IIC_SDA_LOW()   GPIO_ResetBits(IIC_Port, IIC_Pin_SDA)
#define IIC_SDA_READ()  GPIO_ReadInputDataBit(IIC_Port, IIC_Pin_SDA)

/* =======================================================
 * 物理层基础接口 (底层时序)
 * ======================================================= */
void    IIC_Init(void);                         // IIC GPIO 初始化
int     IIC_Start(void);                        // 产生 START 信号
void    IIC_Stop(void);                         // 产生 STOP 信号
void    IIC_Send_Byte(uint8_t Byte);            // 发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);       // 读取一个字节 (ack=1发应答, ack=0发非应答)
void    IIC_Ack(void);                          // 主机发送 ACK
void    IIC_NAck(void);                         // 主机发送 NACK
uint8_t IIC_Wait_Ack(void);                     // 等待从机 ACK (返回0成功, 返回1超时)

/* =======================================================
 * 高级封装接口 (面向设备的寄存器读写)
 * 参数: dev      = 设备 7位地址 (不含读写位)
 *       reg      = 寄存器地址
 *       data/buf = 数据指针
 *       length   = 数据长度
 * ======================================================= */

/**
 * @brief  向指定设备的寄存器写入单字节
 * @retval 1 成功, 0 失败
 */
uint8_t IICwriteByte(uint8_t dev, uint8_t reg, uint8_t data);

/**
 * @brief  向指定设备的寄存器连续写入多字节
 * @retval 1 成功, 0 失败
 */
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);

/**
 * @brief  从指定设备的寄存器读取单字节
 * @retval 1 成功, 0 失败
 */
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data);

/**
 * @brief  从指定设备的寄存器连续读取多字节
 * @retval 读到的字节数
 */
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);

/**
 * @brief  对寄存器中指定的某一位写入 0 或 1
 * @param  bitNum  目标位编号 (0~7, 0为最低位)
 * @param  data    写入值 (0 或非0)
 * @retval 1 成功, 0 失败
 */
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data);

/**
 * @brief  对寄存器中连续几位写入数据 (位域操作)
 * @param  bitStart  起始位编号 (高位，如 bitStart=5,length=3 操作 bit[5:3])
 * @param  length    操作的位数
 * @param  data      写入值 (操作前会自动移位对齐)
 * @retval 1 成功, 0 失败
 */
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);

/* =======================================================
 * 调试工具
 * ======================================================= */
void IIC_Scanf_addr(void);  // 扫描总线上所有在线设备并打印地址

#endif /* __BSP_IIC_H */
