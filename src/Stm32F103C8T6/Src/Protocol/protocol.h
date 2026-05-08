#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include <stdint.h>

#define PACKET_HEADER1 0xFF
#define PACKET_HEADER2_RX 0xFC
#define PACKET_HEADER2_TX 0xFB

#define PROTOCOL_MAX_DATA_LEN 64

typedef struct
{
    uint8_t header1;
    uint8_t header2;
    uint8_t len;
    uint8_t data[PROTOCOL_MAX_DATA_LEN];
    uint8_t checksum;
} Protocol_Packet_t;

typedef void (*Protocol_Callback_t)(Protocol_Packet_t *packet);

void Protocol_Init(Protocol_Callback_t callback);
void Protocol_Process(void);
void Protocol_SendPacket(Protocol_Packet_t *packet);
void Protocol_PackAndSend(uint8_t *pData, uint8_t dataLen);

extern uint32_t protocol_checksum_err_cnt;

#endif
