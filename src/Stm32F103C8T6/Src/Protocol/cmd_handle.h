#ifndef __CMD_HANDLE_H
#define __CMD_HANDLE_H

#include "protocol.h"
#include <stdbool.h>
#include <stdint.h>

// ======================== 控制类型指令 (0x60~0x77) ==========================
#define CMD_RX_EN_CONTROL 0x60           // 脱机/使能控制
#define CMD_RX_VEL_CONTROL 0x61          // 速度模式控制
#define CMD_RX_POS_CONTROL 0x62          // 位置模式控制
#define CMD_RX_STOP_NOW 0x63             // 急停
#define CMD_RX_SYNC_MOTION 0x64          // 多机同步触发
#define CMD_RX_ORIGIN_SET_O 0x65         // 设置单圈回零位置
#define CMD_RX_ORIGIN_MODIFY_PARAMS 0x66 // 修改回零参数
#define CMD_RX_ORIGIN_TRIGGER 0x67       // 触发回零
#define CMD_RX_ORIGIN_INTERRUPT 0x68     // 强制中断回零
#define CMD_RX_MODIFY_CTRL_MODE 0x69     // 修改开环/闭环控制模式
#define CMD_RX_RESET_CUR_POS 0x6A        // 重置当前位置为0
#define CMD_RX_RESET_CLOG_PRO 0x6B       // 解除堵转保护

#define CMD_RX_SERVO_CONTROL 0x6C     // 舵机角度控制
#define CMD_RX_BLDC_CONTROL 0x6D      // 无刷电机转速控制
#define CMD_RX_BLDC_STOP 0x6E         // 无刷电机急停
#define CMD_RX_RGB_SENSOR 0x6F        // 颜色传感器开启或者关闭（预留操作）
#define CMD_RX_SHOW_OLED 0x70         // OLED显示数据
#define CMD_RX_MPU_CALIB 0x71         // MPU校准
#define CMD_RX_MPU_STREAM 0x72        // MPU自动上报开启或者关闭
#define CMD_RX_STEP_MOTOR_STREAM 0x73 // 步进电机自动上报开启或者关闭
#define CMD_RX_PWM_STATE_STREAM 0x74  // 无刷电机占空比、舵机旋转角度自动上报开启或者关闭
#define CMD_RX_RGB_SENSOR_STREAM 0x75 // 颜色传感器数据自动上报或者关闭（预留操作）
#define CMD_RX_SENSOR_STREAM 0x76     // 所有数据数据自动上报或者关闭（预留操作）
#define CMD_OTA_START       0x78     // 进入 OTA 升级模式

// ======================== 查询类型指令 (0x80~0x87) ==========================
#define CMD_RX_QUERY_MPU_ATT 0x80    // 查询MPU6050解算数据（返回俯仰角等）
#define CMD_RX_QUERY_MPU_RAW 0x81    // 查询MPU6050原始数据
#define CMD_RX_QUERY_COLOR_RAW 0x82  // 查询颜色传感器原始数据（预留）
#define CMD_RX_QUERY_COLOR_RES 0x83  // 查询颜色传感器识别结果（预留）
#define CMD_RX_QUERY_SERVO_STAT 0x84 // 查询舵机当前状态
#define CMD_RX_QUERY_BLDC_STAT 0x85  // 查询无刷电机当前状态
#define CMD_RX_QUERY_STEP_STAT 0x86  // 查询步进电机当前状态
#define CMD_RX_QUERY_STEP_PARAM 0x87 // 查询步进电机某个参数状态

// ======================== TX 应答 (0x90~0x93) ==========================
#define CMD_TX_ACK_OK 0x90 // 通用成功应答

// ======================== TX 自动上报功能字 (0x5A~0x5D) ==========================
#define CMD_TX_STREAM_MPU 0x5A   // MPU 自动上报
#define CMD_TX_STREAM_STEP 0x5B  // 步进电机自动上报
#define CMD_TX_STREAM_STATE 0x5C // 无刷/舵机状态自动上报
#define CMD_TX_STREAM_COLOR 0x5D // 颜色传感器自动上报

// ======================== 错误码 ==========================
#define ERR_PARAM_INVALID 0x02

void Cmd_Handle_Init(void);
void Cmd_Handle_Packet_Callback(Protocol_Packet_t *packet);
void Cmd_Handle_Tick(void);

#endif
