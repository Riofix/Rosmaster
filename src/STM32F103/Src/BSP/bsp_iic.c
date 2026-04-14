/**
 * @file    bsp_iic.c
 * @brief   软件模拟 I2C 驱动实现 (GPIO 位bang)
 * @details 物理引脚: SCL -> PB6, SDA -> PB7
 *          提供基础时序接口 + 面向设备的寄存器高级读写接口
 */

#include "bsp_iic.h"
#include "bsp_systick.h"
#include "stdio.h"
#include "stm32f10x.h"

/* -------------------------------------------------------
 * 引脚配置宏 (与 .h 保持一致)
 * ------------------------------------------------------- */
#define IIC_CLK RCC_APB2Periph_GPIOB
#define IIC_Port GPIOB
#define IIC_Pin_SCL GPIO_Pin_6
#define IIC_Pin_SDA GPIO_Pin_7

/* =======================================================
 * 物理层基础接口实现
 * ======================================================= */

/**
 * @brief IIC GPIO 初始化 (SCL/SDA 均为开漏推挽输出, 初始高电平)
 */
void IIC_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_APB2PeriphClockCmd(IIC_CLK, ENABLE);

  GPIO_InitStruct.GPIO_Pin = IIC_Pin_SCL | IIC_Pin_SDA;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(IIC_Port, &GPIO_InitStruct);

  // 释放总线
  IIC_SCL_HIGH();
  IIC_SDA_HIGH();
}

/**
 * @brief 产生 I2C START 信号 (SCL 高时 SDA 下降沿)
 * @retval 始终返回 1
 */
int IIC_Start(void) {
  IIC_SDA_HIGH();
  IIC_SCL_HIGH();
  Delay_us(4);
  IIC_SDA_LOW(); // SCL 高时 SDA 拉低 = START
  Delay_us(4);
  IIC_SCL_LOW(); // 钳住时钟, 准备发送数据
  return 1;
}

/**
 * @brief 产生 I2C STOP 信号 (SCL 高时 SDA 上升沿)
 */
void IIC_Stop(void) {
  IIC_SDA_LOW();
  Delay_us(4);
  IIC_SCL_HIGH();
  Delay_us(4);
  IIC_SDA_HIGH(); // SCL 高时 SDA 拉高 = STOP
  Delay_us(4);
}

/**
 * @brief 等待从机应答信号 ACK
 * @retval 0: 收到 ACK (成功), 1: 超时无 ACK (失败)
 */
uint8_t IIC_Wait_Ack(void) {
  uint8_t err_time = 0;
  IIC_SDA_HIGH();  // 释放SDA，让从机可以拉低
  IIC_SCL_HIGH();
  Delay_us(5);
  while (IIC_SDA_READ()) {
    err_time++;
    if (err_time > 250) {
      IIC_Stop();
      return 1;
    }
    Delay_us(1);
  }
  IIC_SCL_LOW();
  return 0;
}

/**
 * @brief 主机发送 ACK
 */
void IIC_Ack(void) {
  IIC_SCL_LOW();
  IIC_SDA_LOW();
  Delay_us(2);
  IIC_SCL_HIGH();
  Delay_us(2);
  IIC_SCL_LOW();
}

/**
 * @brief 主机发送 NACK (不应答, 用于读操作最后一字节)
 */
void IIC_NAck(void) {
  IIC_SCL_LOW();
  IIC_SDA_HIGH();
  Delay_us(2);
  IIC_SCL_HIGH();
  Delay_us(2);
  IIC_SCL_LOW();
}

/**
 * @brief 发送一个字节 (MSB first)
 * @param Byte 待发送的字节
 */
void IIC_Send_Byte(uint8_t Byte) {
  uint8_t t;
  IIC_SCL_LOW(); // 拉低时钟, 开始数据传输
  for (t = 0; t < 8; t++) {
    if (Byte & 0x80) {
      IIC_SDA_HIGH();
    } else {
      IIC_SDA_LOW();
    }
    Byte <<= 1;
    Delay_us(5);
    IIC_SCL_HIGH();
    Delay_us(5);
    IIC_SCL_LOW();
    Delay_us(5);
  }
}

/**
 * @brief 读取一个字节 (MSB first)
 * @param ack 读完后是否发 ACK: 1=发ACK, 0=发NACK
 * @retval 读到的字节
 */
uint8_t IIC_Read_Byte(unsigned char ack) {
  unsigned char i, receive = 0;
  IIC_SDA_HIGH();  // 释放SDA，准备读取
  for (i = 0; i < 8; i++) {
    IIC_SCL_LOW();
    Delay_us(2);
    IIC_SCL_HIGH();
    receive <<= 1;
    if (IIC_SDA_READ())
      receive++;
    Delay_us(2);
  }
  IIC_SCL_LOW();
  
  if (!ack) {
    IIC_NAck();
  } else {
    IIC_Ack();
  }
  return receive;
}


/* =======================================================
 * 高级封装接口实现
 * dev 参数为 7 位设备地址, 内部发送时左移1位加读写位
 * ======================================================= */

/**
 * @brief  向设备的寄存器写入单字节
 */
uint8_t IICwriteByte(uint8_t dev, uint8_t reg, uint8_t data) {
  return IICwriteBytes(dev, reg, 1, &data);
}

/**
 * @brief  向设备的寄存器连续写多字节
 */
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data) {
  uint8_t i;
  IIC_Start();
  IIC_Send_Byte((dev << 1) | 0); // 写模式
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 0;
  }
  IIC_Send_Byte(reg);
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 0;
  }
  for (i = 0; i < length; i++) {
    IIC_Send_Byte(data[i]);
    if (IIC_Wait_Ack()) {
      IIC_Stop();
      return 0;
    }
  }
  IIC_Stop();
  return 1;
}

/**
 * @brief  从设备的寄存器读取单字节
 */
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data) {
  return IICreadBytes(dev, reg, 1, data);
}

/**
 * @brief  从设备的寄存器连续读取多字节
 * @retval 实际读到的字节数
 */
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data) {
  uint8_t i;
  IIC_Start();
  IIC_Send_Byte((dev << 1) | 0); // 先写模式, 发送寄存器地址
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 0;
  }
  IIC_Send_Byte(reg);
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 0;
  }

  IIC_Start();                   // 重新 START
  IIC_Send_Byte((dev << 1) | 1); // 读模式
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 0;
  }
  for (i = 0; i < length; i++) {
    if (i == length - 1) {
      data[i] = IIC_Read_Byte(0); // 最后一字节发 NACK
    } else {
      data[i] = IIC_Read_Byte(1); // 其余字节发 ACK
    }
  }
  IIC_Stop();
  return length;
}

/**
 * @brief  对寄存器中指定位写入0或1
 */
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data) {
  uint8_t b;
  if (!IICreadByte(dev, reg, &b))
    return 0;
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  return IICwriteByte(dev, reg, b);
}

/**
 * @brief  对寄存器中连续位域写入数据
 */
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length,
                     uint8_t data) {
  uint8_t b;
  if (!IICreadByte(dev, reg, &b))
    return 0;
  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  data <<= (bitStart - length + 1);
  data &= mask;
  b &= ~mask;
  b |= data;
  return IICwriteByte(dev, reg, b);
}

///* =======================================================
// * 调试工具
// * ======================================================= */

///**
// * @brief 扫描 I2C 总线上所有在线设备, 打印其 7 位地址
// */
//void IIC_Scanf_addr(void) {
//  int i2c_count = 0;
//  uint8_t i2c_addr[128]; // 扫描结果数组

//  printf("I2C Bus Scanning...\r\n");
//  for (int i = 1; i < 128; i++) {
//    IIC_Start();
//    IIC_Send_Byte((i << 1) | 0);
//    if (IIC_Wait_Ack() == 0) {
//      // 收到 ACK, 该地址有设备响应
//      i2c_addr[i2c_count] = i;
//      printf("  Found device at 0x%02X (7-bit)\r\n", i);
//      i2c_count++;
//    }
//    IIC_Stop(); // 必须停止以释放总线供下次探测
//  }
//  printf("Scan End, Total Count = %d\r\n", i2c_count);
//}


void IIC_PUSH_DOWN_BUS(void)
{
	IIC_SCL_LOW();
	IIC_SDA_LOW();
}
