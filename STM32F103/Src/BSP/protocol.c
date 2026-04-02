/**
 * @file protocol.c
 * @brief 串口通信协议解析与发送实现文件
 * @details 处理通信协议的接收状态机解析、数据校验以及数据包的组装与发送 (基于 USART2 环形缓冲数据流)
 */

#include "protocol.h"
#include "bsp_usart.h"
#include <string.h>

/**
 * @brief 协议接收状态机枚举
 */
typedef enum {
    STATE_WAIT_HEADER,  // 等待帧头状态
    STATE_GET_CMD,      // 接收命令字状态
    STATE_GET_LEN,      // 接收数据长度状态
    STATE_GET_DATA,     // 接收有效数据状态
    STATE_GET_CHECKSUM  // 接收校验和状态
} ParserState_t;

static ParserState_t state = STATE_WAIT_HEADER;  // 当前接收状态机的状态
static Protocol_Packet_t temp_packet;            // 用于存放接收过程中的数据包缓存
static uint16_t data_cnt = 0;                    // 当前已接收的数据字节数计数器
static Protocol_Callback_t user_handler = 0;     // 协议解析成功后的应用层回调函数
uint32_t protocol_checksum_err_cnt = 0;          // 校验错误统计计数

/**
 * @brief 协议初始化函数
 * @param callback 协议解析成功后调用的用户回调函数
 */
void Protocol_Init(Protocol_Callback_t callback) {
    user_handler = callback;
    protocol_checksum_err_cnt = 0;
}

/**
 * @brief 计算数据包校验和
 * @param p 指向目标数据包的指针
 * @return 计算出的校验和
 */
static uint8_t CalculateChecksum(Protocol_Packet_t* p) {
    uint8_t sum = p->header + p->cmd + p->len;
    for (uint16_t i = 0; i < p->len; i++) {
        sum += p->data[i];
    }
    return sum;
}

/**
 * @brief 协议数据处理轮询函数 (流式处理机制)
 */
void Protocol_Process(void) {
    static uint16_t timeout_ticks = 0;
    
    // 如果没有任何数据，则进行超时判定 (主循环调用间隔约为 10ms左右)
    if (USART2_Available() == 0) {
        if (state != STATE_WAIT_HEADER) {
            timeout_ticks++;
            if (timeout_ticks > 10) { // 超过约 100ms 没完成新包接收，则发生断流，重置状态避免漏包
                state = STATE_WAIT_HEADER;
                timeout_ticks = 0;
            }
        }
        return;
    }

    // 只要底层驱动报告有可读数据，就一直抽取并送入状态机
    while (USART2_Available() > 0) {
        // 读取一个字节，底层会自动推进 read_idx
        uint8_t byte = USART2_ReadByte(); 
        timeout_ticks = 0; // 收到了数据即清零超时判定

        switch (state) {
            case STATE_WAIT_HEADER:
                // 等待接收帧头定义的数据
                if (byte == PACKET_RX_HEADER) {
                    temp_packet.header = byte;
                    state = STATE_GET_CMD;
                }
                break;
            case STATE_GET_CMD:
                // 获取命令字
                temp_packet.cmd = byte;
                state = STATE_GET_LEN;
                break;
            case STATE_GET_LEN:
                // 获取数据长度
                if (byte > PROTOCOL_MAX_DATA_LEN) {
                    // Len上限必须保护！越界即认定此包极其危险，直接抛弃并寻头
                    state = STATE_WAIT_HEADER;
                } else {
                    temp_packet.len = byte;
                    data_cnt = 0;
                    // 若数据长度为0则跳过接收数据阶段直接接收校验和；否则进入数据接收阶段
                    state = (byte == 0) ? STATE_GET_CHECKSUM : STATE_GET_DATA;
                }
                break;
            case STATE_GET_DATA:
                // 接收数据区的数据，增加防越界拦截
                if (data_cnt < PROTOCOL_MAX_DATA_LEN) {
                    temp_packet.data[data_cnt] = byte;
                }
                data_cnt++; // 累加真实接收到的字符数
                
                if (data_cnt >= temp_packet.len) {
                    state = STATE_GET_CHECKSUM; // 数据接收完毕，进入校验和接收阶段
                }
                break;
            case STATE_GET_CHECKSUM:
                // 获取校验和并进行比对
                temp_packet.checksum = byte;
                if (byte == CalculateChecksum(&temp_packet)) {
                    // 校验通过，调用处理该数据包 (通常放入队列)
                    if (user_handler) {
                        user_handler(&temp_packet);
                    }
                } else {
                    // 校验失败记录异常，丢包
                    protocol_checksum_err_cnt++;
                }
                // 无论校验是否通过，解析完一包后复位状态机等待下一包帧头
                state = STATE_WAIT_HEADER;
                break;
        }
    }
}

/**
 * @brief 发送完整的数据包结构体
 */
void Protocol_SendPacket(Protocol_Packet_t* packet) {
    packet->header = PACKET_TX_HEADER;
    packet->checksum = CalculateChecksum(packet);
    
    USART2_SendByte(packet->header);               // 发送发送帧头
    USART2_SendByte(packet->cmd);                  // 发送命令字
    USART2_SendByte(packet->len);                  // 发送数据长度
    USART2_SendBuffer(packet->data, packet->len);  // 连续发送数据区
    USART2_SendByte(packet->checksum);             // 发送校验和
}

/**
 * @brief 快捷打包并发送数据
 */
void Protocol_PackAndSend(uint8_t cmd, uint8_t* pData, uint8_t len) {
    Protocol_Packet_t tx_packet;

    // 自动打包头部与数据信息
    tx_packet.header = PACKET_TX_HEADER;
    tx_packet.cmd = cmd;
    tx_packet.len = len;
    
    // 如果有数据且指针有效，拷贝数据到缓存内，注意不越界
    if(len > 0 && pData != NULL) {
        if(len > PROTOCOL_MAX_DATA_LEN) len = PROTOCOL_MAX_DATA_LEN; // 同样必须防御越界
        tx_packet.len = len;
        memcpy(tx_packet.data, pData, len);
    }

    // 计算校验和
    tx_packet.checksum = CalculateChecksum(&tx_packet);

    // 物理层连续发送完整数据帧
    USART2_SendByte(tx_packet.header);
    USART2_SendByte(tx_packet.cmd);
    USART2_SendByte(tx_packet.len);
    USART2_SendBuffer(tx_packet.data, tx_packet.len);
    USART2_SendByte(tx_packet.checksum);
}
