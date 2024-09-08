#ifndef __ANOPTV8_H__
#define __ANOPTV8_H__
#include <stdint.h>
#include <string.h>
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
协议相关定义，不推荐修改
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ANOPTV8_FRAME_HEAD 0xAB       // 帧头字节
#define ANOPTV8_FRAME_HEADLEN 6       // 帧头功能字节长度，从0xAB开始，后面跟源地址，目标地址，帧ID，数据长度低字节，数据长度高字节，共6字节
#define ANOPTV8_FRAME_MAXDATALEN 200  // 数据区长度，ram小的单片机，不需要发送较长的数据时，可减小长度，需要发送图像等大量数据时，可增大本长度
#define ANOPTV8_FRAME_MAXFRAMELEN ANOPTV8_FRAME_MAXDATALEN + ANOPTV8_FRAME_HEADLEN + 2  // 最大帧长度，帧头功能区长度+数据长度+最后两字节校验
// 字符串颜色
#define ANOLOGCOLOR_DEFAULT 0
#define ANOLOGCOLOR_RED 1
#define ANOLOGCOLOR_GREEN 2
#define ANOLOGCOLOR_BULE 3

// V8协议帧结构定义
typedef struct {
    uint8_t head;
    uint8_t sdevid;
    uint8_t ddevid;
    uint8_t frameid;
    uint16_t datalen;
    uint8_t data[ANOPTV8_FRAME_MAXDATALEN];
    uint8_t sumcheck;
    uint8_t addcheck;
} __attribute__((__packed__)) _st_frame_v8;
// 联合体，可以字节格式访问V8协议帧结构体
typedef union {
    _st_frame_v8 frame;
    uint8_t rawBytes[sizeof(_st_frame_v8)];
} _un_frame_v8;
// 设备ID定义
#define ANOPTV8DEVID_ALL (0xFF)
#define ANOPTV8DEVID_MY (ANOPTV8DEVID_ARB)
#define ANOPTV8DEVID_ARB (0xD2)
#define ANOPTV8DEVID_MCU (0xDB)
#define ANOPTV8DEVID_TKZ (0xDC)
#define ANOPTV8DEVID_LXIMU (0xDD)
#define ANOPTV8DEVID_SWJ (0xFE)
#define ANOPTV8DEVID_OF (0xDE)
#define ANOPTV8DEVID_MV (0xDF)
#define ANOPTV8DEVID_UART1 (0x01)
#define ANOPTV8DEVID_UART2 (0x02)
#define ANOPTV8DEVID_UART3 (0x03)
#define ANOPTV8DEVID_UART4 (0x04)
#define ANOPTV8DEVID_UART5 (0x05)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
设备相关定义，可以修改
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ANOPTV8_HWVER (1)
#define ANOPTV8_SWVER (100)
#define ANOPTV8_BLVER (0)
#define ANOPTV8_PTVER (800)
#define ANOPTV8_DEVNAME "AnoRosBridge"  // DEVNAME：设备名称

#define ANOPTV8_PAR_MAXCOUNT 512
#define ANOPTV8_CMD_MAXCOUNT 128

#define ANOPTV8_MAXSENDFRAME1MS 5
#define ANOPTV8_SENDBUFLEN 100

// 这里只定义硬件的连接方式，为了实现多设备发送，这里用位来定义，最多16种连接方式
#define LT_U1 ((uint16_t)1 << 0)
#define LT_U2 ((uint16_t)1 << 1)
#define LT_U3 ((uint16_t)1 << 2)
#define LT_U4 ((uint16_t)1 << 3)
#define LT_U5 ((uint16_t)1 << 4)
#define LT_UDP_UPPC ((uint16_t)1 << 5)
#define LT_COUNT 6  // 连接方式数量，这里必须与运行匿名协议的连接方式数量对应，使用此数量定义接收缓冲区数量，即一个连接方式对应一个接收缓冲区

#define LT_D_IMU LT_U1
#define LT_D_OF LT_U1
#define LT_D_SWJ LT_UDP_UPPC

#include "AnoPTv8Cmd.hpp"
#include "AnoPTv8ExAPI.hpp"
#include "AnoPTv8FrameFactory.hpp"
#include "AnoPTv8Par.hpp"
#include "AnoPTv8Run.hpp"
#endif
