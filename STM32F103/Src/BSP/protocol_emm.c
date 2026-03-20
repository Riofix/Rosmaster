#include "protocol_emm.h"
#include "bsp_usart.h"

static Emm_Callback_t emm_handler = 0;

void Protocol_Emm_Init(Emm_Callback_t callback) {
    emm_handler = callback;
}

void Protocol_Emm_Process(void) {
    uint8_t raw_buf[32];
    // 从 BSP 获取 USART1 的原始数据包
    uint8_t len = USART1_GetRxData(raw_buf); 
    
    if (len >= 4 && raw_buf[len-1] == 0x6B) { // 简单校验尾部
        Emm_Feedback_t msg;
        msg.addr = raw_buf[0];
        msg.func = raw_buf[1];
        msg.len  = len - 3; // 减去地址、功能码和校验位
        memcpy(msg.data, &raw_buf[2], msg.len);
        
        // 解析完成后，推给 App 层
        if (emm_handler) emm_handler(&msg);
    }
}
