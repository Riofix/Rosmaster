/**
 * @file app_protocol.c
 * @brief STM32 与 RDK X5 的核心业务交互逻辑层 (重构: 无锁队列 + 建表分发)
 */

#include "app_protocol.h"
#include "Emm_V5.h"
#include "bsp_mpu6050.h"
#include "bsp_pwm.h"
#include "msg_queue.h"
#include <stddef.h>

// MPU 自动上报模式标志 (0=关闭, 1=开启)
static uint8_t g_mpu_stream = 0;

// 错误响应通用函数
static void Send_Error_Ack(uint8_t motor_addr, uint8_t failed_cmd,
                           uint8_t err_code) {
  uint8_t tx_data[2];
  tx_data[0] = motor_addr;
  tx_data[1] = err_code;
  Protocol_PackAndSend(CMD_TX_ACK_ERR, tx_data, 2);
}

// 成功响应通用函数
static void Send_Ok_Ack(uint8_t motor_addr, uint8_t ok_cmd) {
  uint8_t tx_data[1];
  tx_data[0] = motor_addr;
  Protocol_PackAndSend(CMD_TX_ACK_OK, tx_data, 1);
}

// ======================= 指令 Handler 函数定义 =======================

// 0x50 脱机/使能控制
static void Handle_En_Control(uint8_t *data, uint8_t len) {
  if (len < 2)
    return;
  Emm_V5_En_Control(data[0], (data[1] != 0), false);
  Send_Ok_Ack(data[0], CMD_RX_EN_CONTROL);
}

// 0x51 速度模式控制
static void Handle_Vel_Control(uint8_t *data, uint8_t len) {
  if (len < 5)
    return;
  uint16_t vel = (data[2] << 8) | data[3];
  Emm_V5_Vel_Control(data[0], data[1], vel, data[4], false);
  Send_Ok_Ack(data[0], CMD_RX_VEL_CONTROL);
}

// 0x52 位置模式控制
static void Handle_Pos_Control(uint8_t *data, uint8_t len) {
  if (len < 10)
    return;
  uint16_t vel = (data[2] << 8) | data[3];
  uint32_t clk = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | data[8];
  Emm_V5_Pos_Control(data[0], data[1], vel, data[4], clk, (data[9] != 0),
                     false);
  Send_Ok_Ack(data[0], CMD_RX_POS_CONTROL);
}

// 0x53 急停
static void Handle_Stop_Now(uint8_t *data, uint8_t len) {
  if (len < 1)
    return;
  Emm_V5_Stop_Now(data[0], false);
  Send_Ok_Ack(data[0], CMD_RX_STOP_NOW);
}

// 0x54 多机同步触发
static void Handle_Sync_Motion(uint8_t *data, uint8_t len) {
  if (len < 1)
    return;
  Emm_V5_Synchronous_motion(data[0]);
  Send_Ok_Ack(data[0], CMD_RX_SYNC_MOTION);
}

// 0x55 设置单圈回零位置
static void Handle_Origin_Set_O(uint8_t *data, uint8_t len) {
  if (len < 2)
    return;
  Emm_V5_Origin_Set_O(data[0], (data[1] != 0));
  Send_Ok_Ack(data[0], CMD_RX_ORIGIN_SET_O);
}

// 0x56 修改回零参数
static void Handle_Origin_Modify_Params(uint8_t *data, uint8_t len) {
  if (len < 12)
    return;
  // uint16_t o_vel = (data[4] << 8) | data[5];
  // uint32_t o_tm = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) |
  // data[9]; 为了简化拆包演示，这里暂时省略详尽解析，您可以按需排布 data
  // Emm_V5_Origin_Modify_Params(...);
  Send_Ok_Ack(data[0], CMD_RX_ORIGIN_MODIFY_PARAMS);
}

// 0x57 触发回零
static void Handle_Origin_Trigger(uint8_t *data, uint8_t len) {
  if (len < 2)
    return;
  Emm_V5_Origin_Trigger_Return(data[0], data[1], false);
  Send_Ok_Ack(data[0], CMD_RX_ORIGIN_TRIGGER);
}

// 0x58 强制中断回零
static void Handle_Origin_Interrupt(uint8_t *data, uint8_t len) {
  if (len < 1)
    return;
  Emm_V5_Origin_Interrupt(data[0]);
  Send_Ok_Ack(data[0], CMD_RX_ORIGIN_INTERRUPT);
}

// 0x59 修改开环/闭环控制模式
static void Handle_Modify_Ctrl_Mode(uint8_t *data, uint8_t len) {
  if (len < 3)
    return;
  Emm_V5_Modify_Ctrl_Mode(data[0], (data[1] != 0), data[2]);
  Send_Ok_Ack(data[0], CMD_RX_MODIFY_CTRL_MODE);
}

// 0x5A 重置当前位置为0
static void Handle_Reset_Cur_Pos(uint8_t *data, uint8_t len) {
  if (len < 1)
    return;
  Emm_V5_Reset_CurPos_To_Zero(data[0]);
  Send_Ok_Ack(data[0], CMD_RX_RESET_CUR_POS);
}

// 0x5B 解除堵转保护
static void Handle_Reset_Clog_Pro(uint8_t *data, uint8_t len) {
  if (len < 1)
    return;
  Emm_V5_Reset_Clog_Pro(data[0]);
  Send_Ok_Ack(data[0], CMD_RX_RESET_CLOG_PRO);
}

// 0x5C 读取系统参数
static void Handle_Read_Sys_Params(uint8_t *data, uint8_t len) {
  if (len < 2)
    return;
  Emm_V5_Read_Sys_Params(data[0], (SysParams_t)data[1]);
  Send_Ok_Ack(data[0], CMD_RX_READ_SYS_PARAMS);
}

//======================= Servo Handler =======================

// 0x70 舵机角度控制  data:[ch(1~2)][angle(0~180)]
static void Handle_SERVO_CONTROL(uint8_t *data, uint8_t len) {
  if (len < 2) {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  PWM_Channel_t ch = (PWM_Channel_t)data[0];
  uint8_t angle = data[1];
  if (ch < PWM_CH2 || ch > PWM_CH3) {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  PWM_Servo_SetAngle(ch, angle);
  uint8_t ok = data[0];
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

// 0x71 舵机脉冲控制  data:[ch(1~2)][pulse_us_H][pulse_us_L]
static void Handle_SERBO_PWM(uint8_t *data, uint8_t len) {
  if (len < 3) {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  PWM_Channel_t ch = (PWM_Channel_t)data[0];
  uint16_t pulse_us = ((uint16_t)data[1] << 8) | data[2];
  if (ch < PWM_CH2 || ch > PWM_CH3) {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  PWM_SetPulse_us(ch, pulse_us);
  uint8_t ok = data[0];
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

//======================= BLDC Handler (PA6 = PWM_CH1 fixed)
//=======================

// 0x72 无刷电机控制  data:[duty(0~100)]
static void Handle_BLDC_CONTROL(uint8_t *data, uint8_t len) {
  if (len < 1) {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  uint8_t duty = data[0];
  PWM_SetDuty(PWM_CH1, duty);
  Protocol_PackAndSend(CMD_TX_ACK_OK, &duty, 1);
}

// 0x73 无刷电机PWM控制  data:[pulse_us_H][pulse_us_L]
static void Handle_BLDC_PWM(uint8_t *data, uint8_t len) {
  if (len < 2) {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  uint16_t pulse_us = ((uint16_t)data[0] << 8) | data[1];
  PWM_SetPulse_us(PWM_CH1, pulse_us);
  uint8_t ok = 0x00;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

// 0x74 无刷电机停止 (PA6 only, servos unaffected)
static void Handle_BLDC_STOP(uint8_t *data, uint8_t len) {
  (void)data;
  (void)len;
  PWM_Stop(PWM_CH1);
  uint8_t ok = 0x00;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

//======================= MPU Handler =======================

// 0x75 读取一次 MPU 数据并立刻上报
static void Handle_MPU_READ(uint8_t *data, uint8_t len) {
  MPU_RawData_t raw;
  if (MPU_ReadRawData(&raw) != 0) {
    Protocol_PackAndSend(CMD_TX_ACK_ERR,
                         (uint8_t *)&(uint8_t){ERR_PARAM_INVALID}, 1);
    return;
  }
  MPU_ComplementaryFilter(&raw, MPU_SAMPLE_DT);
  // 打包 Roll/Pitch/Yaw 三个 float (共12字节) 发给上位机
  Protocol_PackAndSend(CMD_TX_MPU_DATA, (uint8_t *)&g_mpu_attitude,
                       sizeof(MPU_Attitude_t));
}

// 0x76 触发陀螺零偏校准 (约 0.4s 阻塞)
static void Handle_MPU_CALIB(uint8_t *data, uint8_t len) {
  MPU_Calibrate();
  uint8_t ok = 0x00;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

// 0x77 开启 MPU 自动上报
static void Handle_MPU_STREAM_ON(uint8_t *data, uint8_t len) {
  g_mpu_stream = 1;
  uint8_t ok = 0x01;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

// 0x78 关闭 MPU 自动上报
static void Handle_MPU_STREAM_OFF(uint8_t *data, uint8_t len) {
  g_mpu_stream = 0;
  uint8_t ok = 0x00;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}
// ======================= 表驱动核心 =======================

typedef void (*CmdHandler_t)(uint8_t *data, uint8_t len);
typedef struct {
  uint8_t cmd;
  CmdHandler_t handler;
} CmdTable_t;

static const CmdTable_t emm_cmd_table[] = {
    // 1. 控制指令
    {CMD_RX_EN_CONTROL, Handle_En_Control},
    {CMD_RX_VEL_CONTROL, Handle_Vel_Control},
    {CMD_RX_POS_CONTROL, Handle_Pos_Control},
    {CMD_RX_STOP_NOW, Handle_Stop_Now},
    {CMD_RX_SYNC_MOTION, Handle_Sync_Motion},
    {CMD_RX_ORIGIN_SET_O, Handle_Origin_Set_O},
    {CMD_RX_ORIGIN_MODIFY_PARAMS, Handle_Origin_Modify_Params},
    {CMD_RX_ORIGIN_TRIGGER, Handle_Origin_Trigger},
    {CMD_RX_ORIGIN_INTERRUPT, Handle_Origin_Interrupt},
    {CMD_RX_MODIFY_CTRL_MODE, Handle_Modify_Ctrl_Mode},
    {CMD_RX_RESET_CUR_POS, Handle_Reset_Cur_Pos},
    {CMD_RX_RESET_CLOG_PRO, Handle_Reset_Clog_Pro},

    // 2. 查询指令
    {CMD_RX_READ_SYS_PARAMS, Handle_Read_Sys_Params}};
#define CMD_TABLE_SIZE (sizeof(emm_cmd_table) / sizeof(emm_cmd_table[0]))

static const CmdTable_t other_cmd_table[] = {
    // 1. 控制指令
    {CMD_RX_SERVO_CONTROL, Handle_SERVO_CONTROL},
    {CMD_RX_SERBO_PWM, Handle_SERBO_PWM},
    {CMD_RX_BLDC_CONTROL, Handle_BLDC_CONTROL},
    {CMD_RX_BLDC_PWM, Handle_BLDC_PWM},
    {CMD_RX_BLDC_STOP, Handle_BLDC_STOP},
    {CMD_RX_MPU_READ, Handle_MPU_READ},
    {CMD_RX_MPU_CALIB, Handle_MPU_CALIB},
    {CMD_RX_MPU_STREAM_ON, Handle_MPU_STREAM_ON},
    {CMD_RX_MPU_STREAM_OFF, Handle_MPU_STREAM_OFF}};
#define OTHER_CMD_TABLE_SIZE                                                   \
  (sizeof(other_cmd_table) / sizeof(other_cmd_table[0]))

// ======================= 机制实现 =======================

/**
 * @brief 协议初始化，绑定解析回调并初始化队列
 */
void App_Protocol_Init(void) {
  MsgQueue_Init();
  // 注册回调：Protocol解析完的一包会进到下面的 Callback 里
  Protocol_Init(App_Protocol_Packet_Callback);
}

/**
 * @brief 收到完整收发包的处理回调映射中心 (单生产者抛入队列)
 */
void App_Protocol_Packet_Callback(Protocol_Packet_t *packet) {
  // 调用队列系统的线程安全入队方法
  MsgQueue_Enqueue(packet);
}

/**
 * @brief 系统主循环轮询的 App 任务调度 (单消费者拉取与双表驱动分发)
 */
void App_Protocol_Tick(void) {
  Protocol_Packet_t packet;

  // 如果队列中有可用指令包
  if (MsgQueue_Dequeue(&packet)) {

    if (packet.len == 0)
      return;

    bool handled = false;

    // --- 查 EMM 步进电机指令表 (0x50~0x5C) ---
    for (uint16_t i = 0; i < CMD_TABLE_SIZE; i++) {
      if (emm_cmd_table[i].cmd == packet.cmd) {
        if (emm_cmd_table[i].handler != NULL) {
          emm_cmd_table[i].handler(packet.data, packet.len);
        }
        handled = true;
        break;
      }
    }

    // --- 查 其他控制指令表 (0x70~0x78: 舵机/无刷/MPU) ---
    if (!handled) {
      for (uint16_t i = 0; i < OTHER_CMD_TABLE_SIZE; i++) {
        if (other_cmd_table[i].cmd == packet.cmd) {
          if (other_cmd_table[i].handler != NULL) {
            other_cmd_table[i].handler(packet.data, packet.len);
          }
          handled = true;
          break;
        }
      }
    }

    if (!handled) {
      uint8_t m_addr = (packet.len > 0) ? packet.data[0] : 0x00;
      Send_Error_Ack(m_addr, packet.cmd, ERR_UNKNOWN_CMD);
    }
  }

  // ---- MPU 自动上报 (stream 模式, 与主循环 2ms 对齐) ----
  if (g_mpu_stream) {
    MPU_RawData_t raw;
    if (MPU_ReadRawData(&raw) == 0) {
      MPU_ComplementaryFilter(&raw, MPU_SAMPLE_DT);
      Protocol_PackAndSend(CMD_TX_MPU_DATA, (uint8_t *)&g_mpu_attitude,
                           sizeof(MPU_Attitude_t));
    }
  }
}
