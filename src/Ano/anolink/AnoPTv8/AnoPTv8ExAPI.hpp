#ifndef __HARDWAREINTERFACE_H
#define __HARDWAREINTERFACE_H
#include "AnoPTv8.hpp"

void AnoPTv8HwSendBytes(const uint16_t linktype, const uint8_t* buf, const uint16_t len);
void AnoPTv8HwRecvByte(const uint16_t linktype, const uint8_t dat);
void AnoPTv8HwTrigger1ms(void);

void AnoPTv8HwParValRecvCallback(const uint16_t linktype, const _un_frame_v8* p);
void AnoPTv8HwParCmdRecvCallback(const uint16_t linktype, const _un_frame_v8* p);

void AnoPTv8FrameAnl(const uint8_t linktype, const _un_frame_v8* p);

void AnoPTv8FrameExchange(const uint8_t linktype, const _un_frame_v8* p);

#endif
