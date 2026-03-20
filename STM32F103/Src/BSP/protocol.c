#include "protocol.h"
#include "bsp_usart.h"
#include <string.h>

typedef enum {
    STATE_WAIT_HEADER,
    STATE_GET_CMD,
    STATE_GET_LEN,
    STATE_GET_DATA,
    STATE_GET_CHECKSUM
} ParserState_t;

static ParserState_t state = STATE_WAIT_HEADER;
static Protocol_Packet_t temp_packet;
static uint16_t data_cnt = 0;
static uint16_t read_idx = 0;
static Protocol_Callback_t user_handler = 0;

void Protocol_Init(Protocol_Callback_t callback) {
    user_handler = callback;
}

static uint8_t CalculateChecksum(Protocol_Packet_t* p) {
    uint8_t sum = p->header + p->cmd + p->len;
    for (uint16_t i = 0; i < p->len; i++) sum += p->data[i];
    return sum;
}

void Protocol_Process(void) {
    uint8_t* rx_buf = USART2_GetRxBuffer();
    uint16_t write_idx = USART2_GetRxWriteIndex();

    while (read_idx != write_idx) {
        uint8_t byte = rx_buf[read_idx];
        
        switch (state) {
            case STATE_WAIT_HEADER:
                if (byte == PACKET_HEADER) {
                    temp_packet.header = byte;
                    state = STATE_GET_CMD;
                }
                break;
            case STATE_GET_CMD:
                temp_packet.cmd = byte;
                state = STATE_GET_LEN;
                break;
            case STATE_GET_LEN:
                temp_packet.len = byte;
                data_cnt = 0;
                state = (byte == 0) ? STATE_GET_CHECKSUM : STATE_GET_DATA;
                break;
            case STATE_GET_DATA:
                temp_packet.data[data_cnt++] = byte;
                if (data_cnt >= temp_packet.len) state = STATE_GET_CHECKSUM;
                break;
            case STATE_GET_CHECKSUM:
                temp_packet.checksum = byte;
                if (byte == CalculateChecksum(&temp_packet)) {
                    if (user_handler) user_handler(&temp_packet);
                }
                state = STATE_WAIT_HEADER;
                break;
        }
        read_idx = (read_idx + 1) % USART2_RX_BUFFER_SIZE;
    }
}

void Protocol_SendPacket(Protocol_Packet_t* packet) {
    packet->header = PACKET_HEADER;
    packet->checksum = CalculateChecksum(packet);
    USART2_SendByte(packet->header);
    USART2_SendByte(packet->cmd);
    USART2_SendByte(packet->len);
    USART2_SendBuffer(packet->data, packet->len);
    USART2_SendByte(packet->checksum);
}

/**
 * @brief 打包发送函数
 * @param cmd 命令字
 * @param pData 原始数据指针
 * @param len 数据长度
 */
void Protocol_PackAndSend(uint8_t cmd, uint8_t* pData, uint8_t len) {
    Protocol_Packet_t tx_packet;
    
    // 自动打包
    tx_packet.header = PACKET_HEADER;
    tx_packet.cmd = cmd;
    tx_packet.len = len;
    if(len > 0 && pData != NULL) {
        memcpy(tx_packet.data, pData, len);
    }
    
    // 计算校验
    tx_packet.checksum = CalculateChecksum(&tx_packet);
    
    // 物理发送
    USART2_SendByte(tx_packet.header);
    USART2_SendByte(tx_packet.cmd);
    USART2_SendByte(tx_packet.len);
    USART2_SendBuffer(tx_packet.data, tx_packet.len);
    USART2_SendByte(tx_packet.checksum);
}
