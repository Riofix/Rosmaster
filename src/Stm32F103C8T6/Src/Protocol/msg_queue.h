#ifndef __MSG_QUEUE_H
#define __MSG_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#define MSG_QUEUE_BUF_SIZE 8

extern uint32_t drop_err_count;

void MsgQueue_Init(void);
bool MsgQueue_Enqueue(Protocol_Packet_t* packet);
bool MsgQueue_Dequeue(Protocol_Packet_t* packet);

#endif
