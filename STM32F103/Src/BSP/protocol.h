#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include <stdint.h>

#define PACKET_HEADER 0xFF

typedef struct {
    uint8_t header;
    uint8_t cmd;
    uint8_t len;
    uint8_t data[128];
    uint8_t checksum;
} Protocol_Packet_t;

// 定义回调函数类型，当解析到一个完整包时触发
typedef void (*Protocol_Callback_t)(Protocol_Packet_t* packet);

void Protocol_Init(Protocol_Callback_t callback);
void Protocol_Process(void);
void Protocol_SendPacket(Protocol_Packet_t* packet);
void Protocol_PackAndSend(uint8_t cmd, uint8_t* pData, uint8_t len);

#endif
