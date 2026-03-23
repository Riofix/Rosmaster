#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include <stdint.h>

#define PACKET_RX_HEADER 0xFF
#define PACKET_TX_HEADER 0xFB

typedef struct {
    uint8_t header;
    uint8_t id;
    uint8_t cmd;
    uint8_t len;
    uint8_t data[256];
    uint8_t checksum;
} Protocol_Packet_t;

typedef void (*Protocol_Callback_t)(Protocol_Packet_t* packet);

void Protocol_Init(Protocol_Callback_t callback);
void Protocol_Process(void);
void Protocol_SendPacket(Protocol_Packet_t* packet);
void Protocol_PackAndSend(uint8_t id, uint8_t cmd, uint8_t* pData, uint8_t len);

#endif
