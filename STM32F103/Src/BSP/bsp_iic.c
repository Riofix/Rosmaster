#include "stm32f10x.h"
#include "bsp_iic.h"
#include "bsp_systick.h"
#include "stdio.h"

/* 引脚配置宏定义 */
#define IIC_CLK         RCC_APB2Periph_GPIOB
#define IIC_Port        GPIOB
#define IIC_Pin_SCL     GPIO_Pin_6
#define IIC_Pin_SDA     GPIO_Pin_7

/* 延时计数，用于控制IIC通讯频率 */
#define DELAY_FOR_COUNT      10

/* ----------------------------------------------------------------时序底层驱动 ---------------------------------------------------------------- */

/**
 * @Brief: I2C写SDA引脚电平
 * @Note:  当BitValue为0时，置SDA为低电平；当BitValue为1时，置SDA为高电平
 * @Parm:  BitValue: 写入SDA的电平值 (0或1)
 * @Retval: 无
 */
void W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(IIC_Port, IIC_Pin_SDA, (BitAction)!!BitValue);
    Delay_us(DELAY_FOR_COUNT);
}

/**
 * @Brief: I2C写SCL引脚电平
 * @Note:  控制时钟线电平变化，产生通信脉冲
 * @Parm:  BitValue: 写入SCL的电平值 (0或1)
 * @Retval: 无
 */
void W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(IIC_Port, IIC_Pin_SCL, (BitAction)!!BitValue);
    Delay_us(DELAY_FOR_COUNT);
}

/**
 * @Brief: I2C读SDA引脚电平
 * @Note:  用于接收从机数据或判断应答信号
 * @Parm:  无
 * @Retval: 返回当前SDA引脚电平 (0或1)
 */
uint8_t R_SDA(void)
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(IIC_Port, IIC_Pin_SDA);
    Delay_us(DELAY_FOR_COUNT);
    return BitValue;
}

/**
 * @Brief: I2C初始化
 * @Note:  配置GPIO为开漏输出并释放总线（高电平）
 * @Parm:  无
 * @Retval: 无
 */
void IIC_Init(void)
{
    RCC_APB2PeriphClockCmd(IIC_CLK, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // 开漏输出是IIC物理层的要求
    GPIO_InitStructure.GPIO_Pin = IIC_Pin_SCL | IIC_Pin_SDA;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_Port, &GPIO_InitStructure);
    
    // 默认释放总线
    GPIO_SetBits(IIC_Port, IIC_Pin_SCL | IIC_Pin_SDA);
}

/* ----------------------------------------------------------------协议逻辑层 ---------------------------------------------------------------- */

/**
 * @Brief: I2C起始信号
 * @Note:  在SCL高电平期间拉低SDA产生起始条件
 * @Parm:  无
 * @Retval: 1: 成功
 */
int IIC_Start(void)
{
    W_SDA(1);
    W_SCL(1);
    W_SDA(0);
    W_SCL(0);
    return 1;
}

/**
 * @Brief: I2C终止信号
 * @Note:  在SCL高电平期间释放SDA产生停止条件
 * @Parm:  无
 * @Retval: 无
 */
void IIC_Stop(void)
{
    W_SDA(0);
    W_SCL(1);
    W_SDA(1);
}

/**
 * @Brief: I2C发送一个字节
 * @Note:  高位在前(MSB)，逐位发送数据
 * @Parm:  Byte: 要发送的一个字节数据 (0x00~0xFF)
 * @Retval: 无
 */
void IIC_Send_Byte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        W_SDA(!!(Byte & (0x80 >> i)));
        W_SCL(1);
        W_SCL(0);
    }
}

/**
 * @Brief: I2C等待应答位
 * @Note:  主机释放SDA并检测从机是否拉低总线。结束时需将SCL拉低以便后续操作。
 * @Parm:  无
 * @Retval: 1:接收应答成功 | 0:接收应答失败
 */
int IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    W_SDA(1); 
    W_SCL(1);
    while(R_SDA())
    {
        ucErrTime++;
        if (ucErrTime > 50)
        {
            IIC_Stop();
            return 0;
        }
        Delay_us(DELAY_FOR_COUNT);
    }
    W_SCL(0); // 结束应答位，拉低SCL
    return 1;
}

/**
 * @Brief: I2C发送应答位(ACK)
 * @Note:  拉低SDA，通知从机继续发送
 * @Parm:  无
 * @Retval: 无
 */
void IIC_Ack(void)
{
    W_SDA(0);
    W_SCL(1);
    W_SCL(0);
}

/**
 * @Brief: I2C发送非应答位(NACK)
 * @Note:  释放SDA，通知从机停止发送
 * @Parm:  无
 * @Retval: 无
 */
void IIC_NAck(void)
{
    W_SDA(1);
    W_SCL(1);
    W_SCL(0);
}

/**
 * @Brief: I2C读取一个字节
 * @Note:  根据参数选择在读完后发送应答或非应答
 * @Parm:  ack: 1发送ACK，0发送NACK
 * @Retval: 接收到的数据字节
 */
uint8_t IIC_Read_Byte(unsigned char ack)
{
    uint8_t i, Byte = 0x00;
    W_SDA(1); // 接收前释放SDA
    for (i = 0; i < 8; i++)
    {
        W_SCL(1);
        if (R_SDA()) { Byte |= (0x80 >> i); }
        W_SCL(0);
    }
    if (ack) IIC_Ack();
    else     IIC_NAck();
    return Byte;
}

/* ----------------------------------------------------------------应用高级层 ---------------------------------------------------------------- */

/**
 * @Brief: 将多个字节写入指定设备指定寄存器
 * @Note:  dev需传入7位物理地址
 * @Parm:  dev: 设备地址 | reg: 寄存器地址 | length: 数据长度 | data: 指针
 * @Retval: 1:成功 | 0:失败
 */
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t i;
    if (!IIC_Start()) return 0;
    
    IIC_Send_Byte(dev << 1); 
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    
    for (i = 0; i < length; i++)
    {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    }
    IIC_Stop();
    return 1;
}

/**
 * @Brief: 从指定设备指定寄存器读取多个字节
 * @Note:  包含Restart起始信号切换读写模式
 * @Parm:  dev: 设备地址 | reg: 寄存器地址 | length: 长度 | data: 存放指针
 * @Retval: 成功读取的字节数量
 */
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t count = 0;
    if (!IIC_Start()) return 0;
    
    IIC_Send_Byte(dev << 1);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    
    IIC_Start(); 
    IIC_Send_Byte((dev << 1) | 1);
    IIC_Wait_Ack();
    
    for (count = 0; count < length; count++)
    {
        data[count] = IIC_Read_Byte(count != (length - 1));
    }
    IIC_Stop();
    return count;
}

/**
 * @Brief: 写入指定设备指定寄存器一个字节
 * @Note:  单字节写入封装
 * @Parm:  dev: 设备地址 | reg: 寄存器地址 | data: 字节数据
 * @Retval: 1:成功 | 0:失败
 */
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

/**
 * @Brief: 读取指定设备指定寄存器的一个值
 * @Note:  单字节读取封装
 * @Parm:  dev: 设备地址 | reg: 寄存器地址 | data: 数据存放指针
 * @Retval: 1:成功 | 0:失败
 */
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
    return (IICreadBytes(dev, reg, 1, data) == 1);
}

/**
 * @Brief: 读-修改-写 指定字节中的多个位
 * @Note:  常用于配置某些功能寄存器的特定Bit位
 * @Parm:  dev: 设备地址 | reg: 寄存器 | bitStart: 起始位 | length: 长度 | data: 值
 * @Retval: 1:成功 | 0:失败
 */
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t b;
    if (IICreadByte(dev, reg, &b))
    {
        uint8_t mask = ((0xFF << (bitStart + 1)) | (0xFF >> (8 - (bitStart - length + 1))));
        data <<= (bitStart - length + 1);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    }
    return 0;
}

/**
 * @Brief: 读-修改-写 指定字节中的1个位
 * @Note:  简化版的位操作
 * @Parm:  dev: 地址 | reg: 寄存器 | bitNum: 第几位 | data: 0或1
 * @Retval: 1:成功 | 0:失败
 */
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

/**
 * @Brief: 扫描IIC总线上的设备
 * @Note:  遍历7位地址空间(1~127)，发现设备则串口打印提示
 * @Parm:  无
 * @Retval: 无
 */
uint8_t g_addr[256]; // 定义全局扫描结果数组

void IIC_Scanf_addr(void)
{
    uint8_t i2c_count = 0;
    for (int i = 1; i < 128; i++)
    {
        IIC_Start();
        IIC_Send_Byte(i << 1); // 发送写探测
        if (IIC_Wait_Ack())
        {
            printf("External IIC Found address: 0x%02X\n", i);
            i2c_count++;
            IIC_Stop();
        }
        else
        {
            IIC_Stop(); // 必须停止以释放总线供下次探测
        }
    }
    printf("Scan End, Total Count = %d\n", i2c_count);
}
