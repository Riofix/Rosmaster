#include "protocol_emm.h"
#include "bsp_usart.h"
#include <string.h>
#include <stdbool.h>

static Emm_Callback_t emm_handler = 0;

void Protocol_Emm_Init(Emm_Callback_t callback) { emm_handler = callback; }

void Protocol_Emm_Process(void) {
  static uint8_t raw_buf[128];
  static uint8_t len = 0;

  while (USART1_Available() > 0) {
    uint8_t byte = USART1_ReadByte();
    raw_buf[len++] = byte;

    bool is_tail = false;
    // 直接以 0x6B 结尾封包 (长度必须 >= 4 以防误伤)
    if (byte == 0x6B && len >= 4) {
        is_tail = true;
    }
    
    if (is_tail) {
      Emm_Feedback_t msg;
      msg.addr = raw_buf[0];
      msg.func = raw_buf[1];
      msg.len = len - 3; // 去掉 addr, func, 0x6B
      memcpy(msg.data, &raw_buf[2], msg.len);

      if (emm_handler) emm_handler(&msg);
      len = 0;
    }

    if (len >= sizeof(raw_buf)) len = 0;
  }
}
