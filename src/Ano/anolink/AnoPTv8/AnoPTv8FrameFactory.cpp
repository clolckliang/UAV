#include "AnoPTv8FrameFactory.hpp"
#include "AnoPTv8.hpp"
#include "AnoPTv8Cmd.hpp"
#include "AnoPTv8Par.hpp"
#include "AnoPTv8Run.hpp"
// 获取超过一字节数据的每字节数据，例如int16、int32、float等
#define BYTE0(dwTemp) (*((char*)(&dwTemp)))
#define BYTE1(dwTemp) (*((char*)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp) + 3))
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
发送数据时，根据数据帧计算sumcheck和addcheck
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8CalFrameCheck(_un_frame_v8* p) {
    uint8_t sumcheck = 0, addcheck = 0;
    for (uint16_t i = 0; i < (ANOPTV8_FRAME_HEADLEN + p->frame.datalen); i++) {
        sumcheck += p->rawBytes[i];
        addcheck += sumcheck;
    }
    p->rawBytes[ANOPTV8_FRAME_HEADLEN + p->frame.datalen] = sumcheck;
    p->rawBytes[ANOPTV8_FRAME_HEADLEN + p->frame.datalen + 1] = addcheck;
    p->frame.sumcheck = sumcheck;
    p->frame.addcheck = addcheck;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
通用函数，可以方便的发送字节数组
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendBuf(const uint16_t linktype, const uint8_t daddr, const uint8_t fid, const uint8_t* buf, const uint16_t len) {
    uint8_t _cnt = 0;

    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = fid;
        /***********************************************************************/
        for (uint16_t i = 0; i < len; i++) AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = *(buf + i);
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
校验帧发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendCheck(const uint16_t linktype, const uint8_t daddr, const uint8_t id, const uint8_t sc, const uint8_t ac) {
    uint8_t _cnt = 0;
    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x00;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = id;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = sc;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = ac;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
设备信息帧发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendDevInfo(const uint16_t linktype, const uint8_t daddr) {
    uint8_t _cnt = 0;
    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xE3;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = ANOPTV8DEVID_MY;
        uint16_t _tmp = ANOPTV8_HWVER;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
        _tmp = ANOPTV8_SWVER;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
        _tmp = ANOPTV8_BLVER;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
        _tmp = ANOPTV8_PTVER;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
        uint8_t i = 0;
        char* str = ANOPTV8_DEVNAME;
        while (*(str + i) != '\0') {
            AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = *(str + i++);
            if (i > ANOPTV8_FRAME_MAXDATALEN - 20) {
                break;
            }
        }
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
字符串信息发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendStr(const uint16_t linktype, const uint8_t daddr, const uint8_t string_color, const char* str) {
    uint8_t _cnt = 0;
    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xA0;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = string_color;
        uint8_t i = 0;
        while (*(str + i) != '\0') {
            AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = *(str + i++);
            if (i > ANOPTV8_FRAME_MAXDATALEN - 20) {
                break;
            }
        }
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*

*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendValStr(const uint16_t linktype, const uint8_t daddr, const float val, const char* str) {
    uint8_t _cnt = 0;
    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xA1;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(val);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(val);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(val);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(val);
        uint8_t i = 0;
        while (*(str + i) != '\0') {
            AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = *(str + i++);
            if (i > ANOPTV8_FRAME_MAXDATALEN - 20) {
                break;
            }
        }
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
参数类帧发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendParNum(const uint16_t linktype, const uint8_t daddr) {
    uint8_t _cnt = 0;
    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xE0;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = 1;
        uint16_t _tmp = AnoPTv8ParGetCount();
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*

*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendParVal(const uint16_t linktype, const uint8_t daddr, const uint16_t parid) {
    uint8_t _cnt = 0;

    if (parid >= AnoPTv8ParGetCount()) return;

    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xE1;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(parid);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(parid);
        _cnt += AnoPTv8ParCpyVal(parid, &AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt]);
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*

*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendParInfo(const uint16_t linktype, const uint8_t daddr, const uint16_t parid) {
    uint8_t _cnt = 0;

    if (parid >= AnoPTv8ParGetCount()) return;

    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xE2;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(parid);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(parid);
        if (AnoPTv8ParGetInfo(parid) == 0) return;
        const _st_par_info* _p = AnoPTv8ParGetInfo(parid);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = _p->type;

        for (uint8_t i = 0; i < 20; i++) AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = _p->name[i];
        uint8_t i = 0;
        while (_p->info[i] != '\0') {
            AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = _p->info[i++];
            if (i > ANOPTV8_FRAME_MAXDATALEN - 30) {
                break;
            }
        }
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
命令类帧发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendCmdNum(const uint16_t linktype, const uint8_t daddr) {
    uint8_t _cnt = 0;

    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xC1;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = 0;
        uint16_t _tmp = AnoPTv8CmdGetCount();
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*

*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendCmdInfo(const uint16_t linktype, const uint8_t daddr, const uint16_t cmd) {
    uint8_t _cnt = 0;

    if (cmd >= AnoPTv8CmdGetCount()) return;

    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xC2;
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(cmd);
        AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(cmd);

        if (AnoPTv8CmdGetInfo(cmd) == 0) return;
        const _st_cmd_info _p = *AnoPTv8CmdGetInfo(cmd);
        for (int i = 0; i < 3; i++) AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = _p.cmd.cid[i];
        for (int i = 0; i < 8; i++) AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = _p.cmd.valType[i];

        for (uint8_t i = 0; i < 20; i++) AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = _p.name[i];
        uint8_t i = 0;
        while (_p.info[i] != '\0') {
            AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = _p.info[i++];
            if (i > ANOPTV8_FRAME_MAXDATALEN - 20) {
                break;
            }
        }
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
    }
}
