#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "stm32f10x.h"

/* ----------------------------------------------------------------
 * IIC 物理层引脚定义
 * ---------------------------------------------------------------- */
// 如果需要更换引脚，只需修改这里的宏即可
#define IIC_CLK             RCC_APB2Periph_GPIOB
#define IIC_Port            GPIOB
#define IIC_Pin_SCL         GPIO_Pin_6
#define IIC_Pin_SDA         GPIO_Pin_7

/* ----------------------------------------------------------------
 * 底 层 时 序 函 数
 * ---------------------------------------------------------------- */
void IIC_Init(void);                // IIC初始化
int  IIC_Start(void);               // 产生起始信号
void IIC_Stop(void);                // 产生停止信号
void IIC_Send_Byte(uint8_t Byte);   // 发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack); // 读取一个字节 (1:ACK, 0:NACK)
int  IIC_Wait_Ack(void);            // 等待从机应答
void IIC_Ack(void);                 // 主机发送应答
void IIC_NAck(void);                // 主机发送非应答

/* ----------------------------------------------------------------
 * 应 用 层 接 口 函 数
 * ---------------------------------------------------------------- */

/**
 * @Note: 以下所有函数中的 dev 参数均请传入 7位从机地址 (例如 MPU6050 为 0x68)
 * 代码内部会自动进行左移操作并处理读写位。
 */

// 多字节操作
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);

// 单字节操作
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data);

// 位操作 (常用于修改寄存器的特定配置位)
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data);

// 调试工具
void IIC_Scanf_addr(void);          // 扫描总线上所有在线设备

#endif /* __BSP_IIC_H */
