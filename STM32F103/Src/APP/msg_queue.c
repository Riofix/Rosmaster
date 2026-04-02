#include "msg_queue.h"

static Protocol_Packet_t queue_data[MSG_QUEUE_BUF_SIZE];
static volatile uint16_t head_index = 0; // read
static volatile uint16_t tail_index = 0; // write

uint32_t drop_err_count = 0;

void MsgQueue_Init(void) {
    head_index = 0;
    tail_index = 0;
    drop_err_count = 0;
}

// 单生产者: 此处由于是单生产者(Protocol_Process)与单消费者(App_Protocol_Tick)模型，
// 先写入数据，再更新 tail_index 即可实现严格的无锁安全 (Lock-Free)
bool MsgQueue_Enqueue(Protocol_Packet_t* packet) {
    uint16_t next_tail = (tail_index + 1) % MSG_QUEUE_BUF_SIZE;
    if (next_tail == head_index) {
        drop_err_count++; // 队列满，丢弃最新包并做错误计数
        return false;
    }
    
    // 1. 先写数据
    queue_data[tail_index] = *packet;
    
    // 2. 数据写完再移动指针
    tail_index = next_tail;
    return true;
}

// 单消费者
bool MsgQueue_Dequeue(Protocol_Packet_t* packet) {
    if (head_index == tail_index) {
        return false; // 队列空
    }
    
    // 1. 先读取数据
    *packet = queue_data[head_index];
    
    // 2. 再移动读指针
    head_index = (head_index + 1) % MSG_QUEUE_BUF_SIZE;
    return true;
}
