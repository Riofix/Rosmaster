#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

// 默认从机ID
#define DEFAULT_SLAVE_ID 0x0A

// 内部Flash最后几页用于保存参数 
// (F103C8T6 64K Flash -> Page 63: 0x0800FC00 作为泛用安全地址)
#define FLASH_PARAM_ADDR 0x0800FC00

// ======================== RDK X5 -> STM32 接收命令宏定义 ==========================
#define CMD_RX_READ_PARAM 0x50  // 精细参数查询
#define CMD_RX_CTRL_SPEED 0x51  // 速度模式控制
#define CMD_RX_CTRL_POS   0x52  // 位置模式控制
#define CMD_RX_STOP       0x53  // 急停/清除堵转
#define CMD_RX_ENABLE     0x54  // 电机脱机/使能控制
#define CMD_RX_ORIGIN     0x55  // 触发回零
#define CMD_RX_SYNC_RUN   0x56  // 多机同步触发
#define CMD_RX_SET_ID     0x59  // 更改STM32开发板ID

// ======================== STM32 -> RDK X5 发送应答命令宏 ==========================
#define CMD_TX_ACK_PARAM  0x60  // 参数查询返回
#define CMD_TX_ACK_OK     0x61  // 通用握手/成功应答
#define CMD_TX_ACK_ERR    0x62  // 异常报警应答
#define CMD_TX_REPORT_POS 0x63  // 位置达标主动上报

// 错误码定义
#define ERR_FLASH_WRITE   0x01
#define ERR_PARAM_INVALID 0x02
#define ERR_UNKNOWN_CMD   0x03

extern uint8_t g_system_node_id;

void App_Protocol_Init(void);
void App_Protocol_Handler(Protocol_Packet_t* packet);

#endif
