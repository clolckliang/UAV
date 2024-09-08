#ifndef __ANOPTV8RUN_H
#define __ANOPTV8RUN_H
#include "AnoPTv8.hpp"
// 接收缓冲结构体定义
typedef struct {
    uint8_t recvSta;
    uint16_t recvDataLen;
    _un_frame_v8 dataBuf;
} _recvST;

// 发送缓冲结构体定义
typedef struct {
    uint8_t used;
    uint8_t readyToSend;
    uint16_t linkType;
    _un_frame_v8 dataBuf;
} _sendST;

// 发送缓冲区
extern _sendST AnoPTv8TxBuf[ANOPTV8_SENDBUFLEN];

void AnoPTv8RecvOneByte(const uint16_t linktype, const uint8_t dat);
int AnoPTv8GetFreeTxBufIndex(void);
void AnoPTv8RunThread1ms(void);

#endif
