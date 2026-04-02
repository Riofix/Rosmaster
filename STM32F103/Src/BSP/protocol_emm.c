/**
 * @file protocol_emm.c
 * @brief 下位机电机反馈控制协议解析文件 (流式数据组包机制)
 */

#include "protocol_emm.h"
#include "bsp_usart.h"
#include <string.h>

static Emm_Callback_t emm_handler = 0;

void Protocol_Emm_Init(Emm_Callback_t callback) { emm_handler = callback; }

void Protocol_Emm_Process(void) {
  // 用于应对流式断断续续的数据包缓冲
  static uint8_t raw_buf[64];
  static uint8_t len = 0;

  // 只要底层 USART1 驱动报告有数据，就一直接收缓冲
  while (USART1_Available() > 0) {
    uint8_t byte = USART1_ReadByte();
    raw_buf[len++] = byte;

    // Emm协议特点：尾部标识统一为 0x6B。遇到此标识判定可能的一包结束
    if (byte == 0x6B && len >= 4) {
      Emm_Feedback_t msg;
      msg.addr = raw_buf[0];
      msg.func = raw_buf[1];
      msg.len = len - 3; // 减去地址、功能码和校验尾部
      memcpy(msg.data, &raw_buf[2], msg.len);

      // 解析完抛给 App 层处理
      if (emm_handler)
        emm_handler(&msg);

      // 清空缓冲区状态，准备组包下一包
      len = 0;
    }

    // 防止数据异常导致缓冲区溢出
    if (len >= sizeof(raw_buf)) {
      len = 0;
    }
  }
}
