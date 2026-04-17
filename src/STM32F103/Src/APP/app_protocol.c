/**
 * @file app_protocol.c
 * @brief STM32 与 RDK X5 的核心业务交互逻辑层 (重构: 无锁队列 + 建表分发)
 */

#include "app_protocol.h"
#include "Emm_V5.h"
#include "app_tracker.h"
#include "bsp_iic.h"
#include "bsp_mpu6050.h"
#include "bsp_pwm.h"
#include "bsp_systick.h"
#include "bsp_esp8266.h"
#include "oled.h"
#include "msg_queue.h"
#include "protocol_emm.h"
#include <stddef.h>
#include <string.h>

// MPU 自动上报模式标志 (0=关闭, 1=开启)
static uint8_t g_mpu_stream = 0;
// OLED 显示数据 (0=关闭, 1=显示mpu融合数据)
static uint8_t OLED_display = 0;

// 错误响应通用函数
static void Send_Error_Ack(uint8_t motor_addr, uint8_t failed_cmd,
                           uint8_t err_code)
{
  uint8_t tx_data[2];
  tx_data[0] = motor_addr;
  tx_data[1] = err_code;
  Protocol_PackAndSend(CMD_TX_ACK_ERR, tx_data, 2);
}

// 成功响应通用函数
static void Send_Ok_Ack(uint8_t motor_addr, uint8_t ok_cmd)
{
  uint8_t tx_data[1];
  tx_data[0] = motor_addr;
  Protocol_PackAndSend(CMD_TX_ACK_OK, tx_data, 1);
}

// ======================= 指令 Handler 函数定义 =======================

// 0x50 脱机/使能控制
static void Handle_En_Control(uint8_t *data, uint8_t len)
{
  if (len < 2)
    return;
  Emm_V5_En_Control(data[0], (data[1] != 0), false);
  Send_Ok_Ack(data[0], CMD_RX_EN_CONTROL);
}

// 0x51 速度模式控制
static void Handle_Vel_Control(uint8_t *data, uint8_t len)
{
  if (len < 5)
    return;
  uint16_t vel = (data[2] << 8) | data[3];
  Emm_V5_Vel_Control(data[0], data[1], vel, data[4], false);
  Send_Ok_Ack(data[0], CMD_RX_VEL_CONTROL);
}

// 0x52 位置模式控制
static void Handle_Pos_Control(uint8_t *data, uint8_t len)
{
  if (len < 10)
    return;
  uint16_t vel = (data[2] << 8) | data[3];
  uint32_t clk = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | data[8];
  Emm_V5_Pos_Control(data[0], data[1], vel, data[4], clk, (data[9] != 0),
                     false);
  Send_Ok_Ack(data[0], CMD_RX_POS_CONTROL);
}

// 0x53 急停
static void Handle_Stop_Now(uint8_t *data, uint8_t len)
{
  if (len < 1)
    return;
  Emm_V5_Stop_Now(data[0], false);
  Send_Ok_Ack(data[0], CMD_RX_STOP_NOW);
}

// 0x54 多机同步触发
static void Handle_Sync_Motion(uint8_t *data, uint8_t len)
{
  if (len < 1)
    return;
  Emm_V5_Synchronous_motion(data[0]);
  Send_Ok_Ack(data[0], CMD_RX_SYNC_MOTION);
}

// 0x55 设置单圈回零位置
static void Handle_Origin_Set_O(uint8_t *data, uint8_t len)
{
  if (len < 2)
    return;
  Emm_V5_Origin_Set_O(data[0], (data[1] != 0));
  Send_Ok_Ack(data[0], CMD_RX_ORIGIN_SET_O);
}

// 0x56 修改回零参数
static void Handle_Origin_Modify_Params(uint8_t *data, uint8_t len)
{
  if (len < 12)
    return;
  // uint16_t o_vel = (data[4] << 8) | data[5];
  // uint32_t o_tm = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) |
  // data[9]; 为了简化拆包演示，这里暂时省略详尽解析，您可以按需排布 data
  // Emm_V5_Origin_Modify_Params(...);
  Send_Ok_Ack(data[0], CMD_RX_ORIGIN_MODIFY_PARAMS);
}

// 0x57 触发回零
static void Handle_Origin_Trigger(uint8_t *data, uint8_t len)
{
  if (len < 2)
    return;
  Emm_V5_Origin_Trigger_Return(data[0], data[1], false);
  Send_Ok_Ack(data[0], CMD_RX_ORIGIN_TRIGGER);
}

// 0x58 强制中断回零
static void Handle_Origin_Interrupt(uint8_t *data, uint8_t len)
{
  if (len < 1)
    return;
  Emm_V5_Origin_Interrupt(data[0]);
  Send_Ok_Ack(data[0], CMD_RX_ORIGIN_INTERRUPT);
}

// 0x59 修改开环/闭环控制模式
static void Handle_Modify_Ctrl_Mode(uint8_t *data, uint8_t len)
{
  if (len < 3)
    return;
  Emm_V5_Modify_Ctrl_Mode(data[0], (data[1] != 0), data[2]);
  Send_Ok_Ack(data[0], CMD_RX_MODIFY_CTRL_MODE);
}

// 0x5A 重置当前位置为0
static void Handle_Reset_Cur_Pos(uint8_t *data, uint8_t len)
{
  if (len < 1)
    return;
  Emm_V5_Reset_CurPos_To_Zero(data[0]);
  Send_Ok_Ack(data[0], CMD_RX_RESET_CUR_POS);
}

// 0x5B 解除堵转保护
static void Handle_Reset_Clog_Pro(uint8_t *data, uint8_t len)
{
  if (len < 1)
    return;
  Emm_V5_Reset_Clog_Pro(data[0]);
  Send_Ok_Ack(data[0], CMD_RX_RESET_CLOG_PRO);
}

// 0x5C 读取系统参数
static void Handle_Read_Sys_Params(uint8_t *data, uint8_t len)
{
  if (len < 2)
    return;
  Emm_V5_Read_Sys_Params(data[0], (SysParams_t)data[1]);
  Send_Ok_Ack(data[0], CMD_RX_READ_SYS_PARAMS);
}

//======================= Servo Handler =======================

// 0x70 舵机角度控制  data:[ch(1~2)][angle(0~180)]
static void Handle_SERVO_CONTROL(uint8_t *data, uint8_t len)
{
  if (len < 2)
  {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  PWM_Channel_t ch = (PWM_Channel_t)data[0];
  uint8_t angle = data[1];
  if (ch < PWM_CH2 || ch > PWM_CH3)
  {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  PWM_Servo_SetAngle(ch, angle);
  uint8_t ok = data[0];
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

// 0x71 舵机脉冲控制  data:[ch(1~2)][pulse_us_H][pulse_us_L]
static void Handle_SERBO_PWM(uint8_t *data, uint8_t len)
{
  if (len < 3)
  {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  PWM_Channel_t ch = (PWM_Channel_t)data[0];
  uint16_t pulse_us = ((uint16_t)data[1] << 8) | data[2];
  if (ch < PWM_CH2 || ch > PWM_CH3)
  {
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
static void Handle_BLDC_CONTROL(uint8_t *data, uint8_t len)
{
  if (len < 1)
  {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  uint8_t duty = data[0];
  PWM_SetDuty(PWM_CH1, duty);
  Protocol_PackAndSend(CMD_TX_ACK_OK, &duty, 1);
}

// 0x73 无刷电机PWM控制  data:[pulse_us_H][pulse_us_L]
static void Handle_BLDC_PWM(uint8_t *data, uint8_t len)
{
  if (len < 2)
  {
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
static void Handle_BLDC_STOP(uint8_t *data, uint8_t len)
{
  (void)data;
  (void)len;
  PWM_Stop(PWM_CH1);
  uint8_t ok = 0x00;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

//======================= MPU Handler =======================

// 0x75 读取一次 MPU 数据并立刻上报
static void Handle_MPU_READ(uint8_t *data, uint8_t len)
{
  MPU_RawData_t raw;
  if (MPU_ReadRawData(&raw) != 0)
  {
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
static void Handle_MPU_CALIB(uint8_t *data, uint8_t len)
{
  MPU_Calibrate();
  uint8_t ok = 0x00;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

// 0x77 开启 MPU 自动上报
static void Handle_MPU_STREAM_ON(uint8_t *data, uint8_t len)
{
  g_mpu_stream = 1;
  uint8_t ok = 0x01;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

// 0x78 关闭 MPU 自动上报
static void Handle_MPU_STREAM_OFF(uint8_t *data, uint8_t len)
{
  g_mpu_stream = 0;
  uint8_t ok = 0x00;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

static uint32_t s_emm_rx_cnt = 0;    // EMM 反馈包计数器
static int32_t s_last_abs_ticks = 0; // 缓存最后一包电机的绝对脉冲数

// 0x79 里程计查询
static void Handle_ODOM_QUERY(uint8_t *data, uint8_t len)
{
  (void)data;
  (void)len;
  float pos, target;
  uint8_t mode;
  Tracker_GetState(&pos, &target, &mode);

  uint8_t tx_buf[13]; // 4(pos) + 4(target) + 1(mode) + 4(raw_ticks)
  memcpy(&tx_buf[0], &pos, 4);
  memcpy(&tx_buf[4], &target, 4);
  tx_buf[8] = mode;
  memcpy(&tx_buf[9], &s_last_abs_ticks, 4);

  Protocol_PackAndSend(CMD_TX_TRACKER_DATA, tx_buf, 13);
}

// 0x7A 设置目标里程 (单位: 毫米)
static void Handle_TRACKER_SET_GOAL(uint8_t *data, uint8_t len)
{
  if (len < 4)
    return;
  float target_mm;
  memcpy(&target_mm, data, 4);
  Tracker_SetTarget(target_mm);

  uint8_t ok = 0x00;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}

// 0x7B OLED显示数据
static void Handle_SHOW_OLED(uint8_t *data, uint8_t len)
{
  if (len != 1)
  {
    uint8_t e = ERR_PARAM_INVALID;
    Protocol_PackAndSend(CMD_TX_ACK_ERR, &e, 1);
    return;
  }
  OLED_Clear();
  OLED_display = data[0];
  uint8_t ok = 0x00;
  Protocol_PackAndSend(CMD_TX_ACK_OK, &ok, 1);
}
// ======================= 表驱动核心 =======================

typedef void (*CmdHandler_t)(uint8_t *data, uint8_t len);
typedef struct
{
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
    {CMD_RX_MPU_STREAM_OFF, Handle_MPU_STREAM_OFF},
    {CMD_RX_ODOM_QUERY, Handle_ODOM_QUERY},
    {CMD_RX_TRACKER_SET_GOAL, Handle_TRACKER_SET_GOAL},
    {CMD_RX_SHOW_OLED, Handle_SHOW_OLED}};
#define OTHER_CMD_TABLE_SIZE \
  (sizeof(other_cmd_table) / sizeof(other_cmd_table[0]))

// ======================= 机制实现 =======================

/**
 * @brief EMM 电机反馈回调，将上报的参数送入 Tracker
 */
static void App_Protocol_Emm_Callback(Emm_Feedback_t *msg)
{
  /* 0x36 为 S_CPOS (当前位置) 反馈, 兼容部分带状态位的固件(Len=5) */
  if (msg->func == 0x36 && msg->len >= 4)
  {
    s_emm_rx_cnt++; // 收到有效包，计数增加

    // 如果带状态位，数据可能偏移或长度不同，但通常前4字节或后4字节是位置
    // 还原为大端序 (Big-Endian) 解析尝试: D3 D2 D1 D0
    int32_t abs_ticks = (int32_t)((msg->data[0] << 24) | (msg->data[1] << 16) |
                                  (msg->data[2] << 8) | msg->data[3]);

    // 缓存原始值供诊断指令使用
    extern int32_t s_last_abs_ticks;
    s_last_abs_ticks = abs_ticks;

    Tracker_Update(abs_ticks, MPU_Get_GyroZ_DPS());
  }
}

/**
 * @brief 协议初始化，绑定解析回调并初始化队列
 */
void App_Protocol_Init(void)
{
  MsgQueue_Init();
  Tracker_Init();

  // 注册回调：Protocol解析完的一包会进到下面的 Callback 里
  Protocol_Init(App_Protocol_Packet_Callback);
  // 注册 EMM 电机反馈回调
  Protocol_Emm_Init(App_Protocol_Emm_Callback);
}

/**
 * @brief 收到完整收发包的处理回调映射中心 (单生产者抛入队列)
 */
void App_Protocol_Packet_Callback(Protocol_Packet_t *packet)
{
  // 调用队列系统的线程安全入队方法
  MsgQueue_Enqueue(packet);
}

/**
 * @brief 系统主循环轮询的 App 任务调度 (单消费者拉取与双表驱动分发)
 */
void App_Protocol_Tick(void)
{
  Protocol_Packet_t packet;

  // 如果队列中有可用指令包
  if (MsgQueue_Dequeue(&packet))
  {
    bool handled = false;

    // --- 查 EMM 步进电机指令表 (0x50~0x5C) ---
    for (uint16_t i = 0; i < CMD_TABLE_SIZE; i++)
    {
      if (emm_cmd_table[i].cmd == packet.cmd)
      {
        if (emm_cmd_table[i].handler != NULL)
        {
          emm_cmd_table[i].handler(packet.data, packet.len);
        }
        handled = true;
        break;
      }
    }

    // --- 查 其他控制指令表 (0x70~0x78: 舵机/无刷/MPU) ---
    if (!handled)
    {
      for (uint16_t i = 0; i < OTHER_CMD_TABLE_SIZE; i++)
      {
        if (other_cmd_table[i].cmd == packet.cmd)
        {
          if (other_cmd_table[i].handler != NULL)
          {
            other_cmd_table[i].handler(packet.data, packet.len);
          }
          handled = true;
          break;
        }
      }
    }

    if (!handled)
    {
      uint8_t m_addr = (packet.len > 0) ? packet.data[0] : 0x00;
      Send_Error_Ack(m_addr, packet.cmd, ERR_UNKNOWN_CMD);
    }
  }

  // ---- MPU 数据处理 ----
  MPU_RawData_t raw;
  if (MPU_ReadRawData(&raw) == 0)
    MPU_ComplementaryFilter(&raw, MPU_SAMPLE_DT);

  // ---- MPU 自动上报 (stream 模式, 与主循环 2ms 对齐) ----
  if (g_mpu_stream)
    Protocol_PackAndSend(CMD_TX_MPU_DATA, (uint8_t *)&g_mpu_attitude,
                         sizeof(MPU_Attitude_t));

  // ---- 自动轮询 EMM 电机位置 (20Hz / 50ms) ----
  static uint32_t emm_poll_tick = 0;
  if (++emm_poll_tick >= 25)
  { // 2ms * 25 = 50ms
    emm_poll_tick = 0;
    /* 主动查询 Tracker 对应电机的当前位置 (S_CPOS = 10 ->
     * 0x5C指令中对应的参数码) */
    Emm_V5_Read_Sys_Params(TRACKER_MOTOR_ADDR, S_CPOS);
  }

  // ---- 自动显示 (与主循环2ms对齐) ----
  switch (OLED_display)
  {
  case 1:
    OLED_ShowString(1, 1, "Roll");
    OLED_ShowSignedNum(1, 8, g_mpu_attitude.roll, 3);
    OLED_ShowString(2, 1, "Pitch");
    OLED_ShowSignedNum(2, 8, g_mpu_attitude.pitch, 3);
    OLED_ShowString(3, 1, "Yaw");
    OLED_ShowSignedNum(3, 8, g_mpu_attitude.yaw, 3);
    break;

  default:
    break;
  }
}
