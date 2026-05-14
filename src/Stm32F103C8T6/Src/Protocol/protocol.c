#include "protocol.h"
#include "bsp_usart.h"
#include <string.h>

/**
 * @brief 协议接收状态机枚举
 */
typedef enum
{
  STATE_WAIT_HEADER1, // 等待第一帧头 0xFF
  STATE_WAIT_HEADER2, // 等待第二帧头 0xFC
  STATE_GET_LEN,      // 接收长度字节
  STATE_GET_DATA,     // 接收有效数据
  STATE_GET_CHECKSUM  // 接收校验位
} ParserState_t;

static ParserState_t state = STATE_WAIT_HEADER1;
static Protocol_Packet_t temp_packet;
static uint16_t data_cnt = 0;
static Protocol_Callback_t user_handler = 0;
uint32_t protocol_checksum_err_cnt = 0;

/**
 * @brief 初始化协议栈
 * @param callback 接收到完整数据包后的回调函数
 */
void Protocol_Init(Protocol_Callback_t callback)
{
  user_handler = callback;
  protocol_checksum_err_cnt = 0;
}

/**
 * @brief 计算校验和
 * 从长度位开始累加到校验位前
 * @param p 数据包指针
 * @return 校验和
 */
static uint8_t CalculateChecksum(Protocol_Packet_t *p)
{
  uint8_t sum = p->len;
  uint8_t data_len = p->len - 2; // 长度 = len(1) + data + check(1)
  for (uint16_t i = 0; i < data_len; i++)
  {
    sum += p->data[i];
  }
  return sum;
}

/**
 * @brief 协议处理主函数 (状态机)
 * 需在主循环中周期性调用
 */
void Protocol_Process(void)
{
  static uint16_t timeout_ticks = 0;

  if (USART2_Available() == 0)
  {
    if (state != STATE_WAIT_HEADER1)
    {
      timeout_ticks++;
      if (timeout_ticks > 10)
      {
        state = STATE_WAIT_HEADER1;
        timeout_ticks = 0;
      }
    }
    return;
  }

  while (USART2_Available() > 0)
  {
    uint8_t byte = USART2_ReadByte();
    timeout_ticks = 0;

    switch (state)
    {
    case STATE_WAIT_HEADER1:
      if (byte == PACKET_HEADER1)
      {
        temp_packet.header1 = byte;
        state = STATE_WAIT_HEADER2;
      }
      break;

    case STATE_WAIT_HEADER2:
      if (byte == PACKET_HEADER2_RX)
      {
        temp_packet.header2 = byte;
        state = STATE_GET_LEN;
      }
      else if (byte == PACKET_HEADER1)
      {
        // 容错：如果是连续的 0xFF，保持在等待 HEADER2 状态
        state = STATE_WAIT_HEADER2;
      }
      else
      {
        state = STATE_WAIT_HEADER1;
      }
      break;

    case STATE_GET_LEN:
      // 长度必须至少为 2 (包含 len 和 checksum 自身)
      if (byte < 2 || byte > (PROTOCOL_MAX_DATA_LEN + 2))
      {
        state = STATE_WAIT_HEADER1;
      }
      else
      {
        temp_packet.len = byte;
        data_cnt = 0;
        // 如果 len 为 2，说明没有数据位，直接进入收校验位状态
        state = (byte == 2) ? STATE_GET_CHECKSUM : STATE_GET_DATA;
      }
      break;

    case STATE_GET_DATA:
      temp_packet.data[data_cnt++] = byte;
      // 数据长度 = len - 2
      if (data_cnt >= (temp_packet.len - 2))
      {
        state = STATE_GET_CHECKSUM;
      }
      break;

    case STATE_GET_CHECKSUM:
      temp_packet.checksum = byte;
      if (byte == CalculateChecksum(&temp_packet))
      {
        if (user_handler)
        {
          user_handler(&temp_packet);
        }
      }
      else
      {
        protocol_checksum_err_cnt++;
      }
      state = STATE_WAIT_HEADER1;
      break;
    }
  }
}

/**
 * @brief 发送完整数据包
 */
void Protocol_SendPacket(Protocol_Packet_t *packet)
{
  packet->header1 = PACKET_HEADER1;
  packet->header2 = PACKET_HEADER2_TX;
  packet->checksum = CalculateChecksum(packet);

  USART2_SendByte(packet->header1);
  USART2_SendByte(packet->header2);
  USART2_SendByte(packet->len);
  USART2_SendBuffer(packet->data, packet->len - 2);
  USART2_SendByte(packet->checksum);
}

/**
 * @brief 打包并发送数据 (自动处理 Len 和 Checksum)
 */
void Protocol_PackAndSend(uint8_t *pData, uint8_t dataLen)
{
  if (dataLen > PROTOCOL_MAX_DATA_LEN)
    dataLen = PROTOCOL_MAX_DATA_LEN;

  Protocol_Packet_t tx_packet;
  tx_packet.header1 = PACKET_HEADER1;
  tx_packet.header2 = PACKET_HEADER2_TX;
  tx_packet.len = dataLen + 2; // len位 + 数据长度 + 校验位

  if (dataLen > 0 && pData != NULL)
  {
    memcpy(tx_packet.data, pData, dataLen);
  }

  tx_packet.checksum = CalculateChecksum(&tx_packet);

  USART2_SendByte(tx_packet.header1);
  USART2_SendByte(tx_packet.header2);
  USART2_SendByte(tx_packet.len);
  if (dataLen > 0)
  {
    USART2_SendBuffer(tx_packet.data, dataLen);
  }
  USART2_SendByte(tx_packet.checksum);
}
