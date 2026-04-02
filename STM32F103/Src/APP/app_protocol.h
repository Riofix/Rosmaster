#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "../BSP/protocol.h"

// ======================== RDK X5 -> STM32 接收命令宏定义 (13条) ==========================
// ----- 1. 控制指令 ----- 
#define CMD_RX_EN_CONTROL           0x50  // 脱机/使能控制
#define CMD_RX_VEL_CONTROL          0x51  // 速度模式控制
#define CMD_RX_POS_CONTROL          0x52  // 位置模式控制
#define CMD_RX_STOP_NOW             0x53  // 急停
#define CMD_RX_SYNC_MOTION          0x54  // 多机同步触发
#define CMD_RX_ORIGIN_SET_O         0x55  // 设置单圈回零位置
#define CMD_RX_ORIGIN_MODIFY_PARAMS 0x56  // 修改回零参数
#define CMD_RX_ORIGIN_TRIGGER       0x57  // 触发回零
#define CMD_RX_ORIGIN_INTERRUPT     0x58  // 强制中断回零
#define CMD_RX_MODIFY_CTRL_MODE     0x59  // 修改开环/闭环控制模式
#define CMD_RX_RESET_CUR_POS        0x5A  // 重置当前位置为0
#define CMD_RX_RESET_CLOG_PRO       0x5B  // 解除堵转保护

// ----- 2. 查询指令 -----
#define CMD_RX_READ_SYS_PARAMS      0x5C  // 读取系统参数

// ======================== STM32 -> RDK X5 发送应答命令宏 ==========================
#define CMD_TX_ACK_PARAM  0x60  // 参数查询返回
#define CMD_TX_ACK_OK     0x61  // 通用握手/成功应答
#define CMD_TX_ACK_ERR    0x62  // 异常报警应答
#define CMD_TX_REPORT_POS 0x63  // 位置达标主动上报

// 错误码定义
#define ERR_FLASH_WRITE   0x01
#define ERR_PARAM_INVALID 0x02
#define ERR_UNKNOWN_CMD   0x03

void App_Protocol_Init(void);
void App_Protocol_Packet_Callback(Protocol_Packet_t* packet); // 解析完毕回调
void App_Protocol_Tick(void); // 放在 main 循环的主轮询函数

#endif
