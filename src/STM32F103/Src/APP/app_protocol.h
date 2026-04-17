#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

#include "../BSP/protocol.h"
#include "../BSP/bsp_mpu6050.h"
#include <stdbool.h>
#include <stdint.h>

// ======================== RDK X5 -> STM32 接收命令宏定义 (13条)
// 步进电机控制==========================
// ----- 1. 控制指令 -----
#define CMD_RX_EN_CONTROL 0x50           // 脱机/使能控制
#define CMD_RX_VEL_CONTROL 0x51          // 速度模式控制
#define CMD_RX_POS_CONTROL 0x52          // 位置模式控制
#define CMD_RX_STOP_NOW 0x53             // 急停
#define CMD_RX_SYNC_MOTION 0x54          // 多机同步触发
#define CMD_RX_ORIGIN_SET_O 0x55         // 设置单圈回零位置
#define CMD_RX_ORIGIN_MODIFY_PARAMS 0x56 // 修改回零参数
#define CMD_RX_ORIGIN_TRIGGER 0x57       // 触发回零
#define CMD_RX_ORIGIN_INTERRUPT 0x58     // 强制中断回零
#define CMD_RX_MODIFY_CTRL_MODE 0x59     // 修改开环/闭环控制模式
#define CMD_RX_RESET_CUR_POS 0x5A        // 重置当前位置为0
#define CMD_RX_RESET_CLOG_PRO 0x5B       // 解除堵转保护

// ----- 2. 查询指令 -----
#define CMD_RX_READ_SYS_PARAMS 0x5C // 读取系统参数

// ======================== RDK X5 -> STM32 接收命令宏定义 (9条)
// 舵机、MPU、无刷电机控制==========================
#define CMD_RX_SERVO_CONTROL 0x70  // 舵机角度控制
#define CMD_RX_SERBO_PWM 0x71      // 舵机PWM控制
#define CMD_RX_BLDC_CONTROL 0x72   // 无刷电机控制
#define CMD_RX_BLDC_PWM 0x73       // 无刷电机PWM控制
#define CMD_RX_BLDC_STOP 0x74      // 无刷电机急停
#define CMD_RX_MPU_READ 0x75       // MPU读取并发送数据
#define CMD_RX_MPU_CALIB 0x76      // MPU校准
#define CMD_RX_MPU_STREAM_ON 0x77  // MPU自动上报开启
#define CMD_RX_MPU_STREAM_OFF 0x78 // MPU自动上报关闭
#define CMD_RX_ODOM_QUERY     0x79 // 里程计查询
#define CMD_RX_TRACKER_SET_GOAL 0x7A // 设置追踪目标位置
#define CMD_RX_SHOW_OLED 0x7B				// OLED显示数据


// ======================== STM32 -> RDK X5 发送应答命令宏 ==========================
#define CMD_TX_ACK_PARAM  0x60  // 参数查询返回
#define CMD_TX_ACK_OK     0x61  // 通用握手/成功应答
#define CMD_TX_ACK_ERR    0x62  // 异常报警应答
#define CMD_TX_REPORT_POS 0x63  // 位置达标主动上报
#define CMD_TX_MPU_DATA   0x64  // MPU 姿态数据上报 (Roll/Pitch/Yaw, 各4字节float, 共12字节)
#define CMD_TX_TRACKER_DATA 0x65 // 里程计上报 (pos/target 各4字节float, mode 1字节)

// 错误码定义
#define ERR_FLASH_WRITE 0x01       // Flash 写入错误
#define ERR_PARAM_INVALID 0x02     // 参数无效
#define ERR_UNKNOWN_CMD 0x03       // 未知命令

void App_Protocol_Init(void);
void App_Protocol_Packet_Callback(Protocol_Packet_t *packet); // 解析完毕回调
void App_Protocol_Tick(void); // 放在 main 循环的主轮询函数

#endif
