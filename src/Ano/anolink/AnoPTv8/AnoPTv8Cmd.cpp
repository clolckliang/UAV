#include "AnoPTv8Cmd.hpp"
// 定义一个指针数组，指针指向定义的命令信息结构体
const _st_cmd_info* pCmdInfoList[ANOPTV8_CMD_MAXCOUNT];
// 命令个数，每注册一个命令，个数+1
int cmdCount = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
命令解析
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8CmdFrameAnl(const uint16_t linktype, const _un_frame_v8* p) {
    switch (p->frame.frameid) {
        case 0xC0:
            // 收到命令，根据前3字节的命令ID，搜索是哪条命令，然后根据该命令的函数指针，执行命令函数
            for (uint16_t i = 0; i < AnoPTv8CmdGetCount(); i++) {
                _st_cmd_info _ci = *AnoPTv8CmdGetInfo(i);
                if (p->frame.data[0] == _ci.cmd.cid[0] && p->frame.data[1] == _ci.cmd.cid[1] && p->frame.data[2] == _ci.cmd.cid[2]) {
                    // 返回校验帧，代表本命令执行成功
                    AnoPTv8SendCheck(linktype, p->frame.sdevid, p->frame.frameid, p->frame.sumcheck, p->frame.addcheck);
                    // 根据该命令的函数指针，执行命令函数
                    _ci.pFun(p, i);
                    return;
                }
            }
            break;
        case 0xC1:
            // CMD读取命令信息
            if (p->frame.data[0] == 0) {
                // 读取CMD个数
                AnoPTv8SendCmdNum(linktype, p->frame.sdevid);
            } else if (p->frame.data[0] == 1) {
                // 读取CMD信息
                AnoPTv8SendCmdInfo(linktype, p->frame.sdevid, *(uint16_t*)(p->frame.data + 1));
            }
            break;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
获取已注册的命令个数
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int AnoPTv8CmdGetCount(void) { return cmdCount; }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
命令注册函数
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8CmdRegister(const _st_cmd_info* _pi) {
    if (cmdCount <= (ANOPTV8_CMD_MAXCOUNT - 1)) {
        pCmdInfoList[cmdCount++] = _pi;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
根据index获取命令信息
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const _st_cmd_info* AnoPTv8CmdGetInfo(const uint16_t cmdindex) {
    if (cmdindex > AnoPTv8CmdGetCount()) return 0;
    return pCmdInfoList[cmdindex];
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
获取本命令所有参数的总长度
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int AnoPTv8CmdGetValsLength(const uint16_t cmdindex) {
    if (cmdindex > AnoPTv8CmdGetCount()) return -1;
    int _len = 0;
    for (int i = 0; i < 8; i++) {
        if (pCmdInfoList[cmdindex]->cmd.valType[i] == 0) break;
        switch (pCmdInfoList[cmdindex]->cmd.valType[i]) {
            case CVT_U8:
            case CVT_S8:
                _len += 1;
                break;
            case CVT_U16:
            case CVT_S16:
                _len += 2;
                break;
            case CVT_U32:
            case CVT_S32:
            case CVT_Float:
                _len += 4;
                break;
            case CVT_U64:
            case CVT_S64:
            case CVT_Double:
                _len += 8;
                break;
            default:
                break;
        }
    }
    return _len;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
对命令数据帧进行解析，获取某个参数的参数值
第一个参数为该参数的地址，支持的参数格式及定义见eAnoPTv8_cmdValType定义
第二个参数为数据帧的指针
第三个参数为命令index，对应命令注册时的顺序
第四个参数为需要获取的参数的index
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8CmdValCpy(void* d, const _un_frame_v8* p, const uint16_t cmdindex, const int valindex) {
    // 判断参数有效性
    if (cmdindex > AnoPTv8CmdGetCount()) return;
    if (valindex >= 8) return;
    // 获取本参数的长度
    uint16_t _vallen = 0;
    switch (pCmdInfoList[cmdindex]->cmd.valType[valindex]) {
        case CVT_U8:
        case CVT_S8:
            _vallen = 1;
            break;
        case CVT_U16:
        case CVT_S16:
            _vallen = 2;
            break;
        case CVT_U32:
        case CVT_S32:
        case CVT_Float:
            _vallen = 4;
            break;
        case CVT_U64:
        case CVT_S64:
        case CVT_Double:
            _vallen = 8;
            break;
        default:
            break;
    }
    // 获取本参数相对数据帧内参数区域起始位置的偏移，也就是计算本参数前面所有参数的长度和
    uint16_t _valaddr = 0;
    for (int i = 0; i < valindex; i++) {
        switch (pCmdInfoList[cmdindex]->cmd.valType[i]) {
            case CVT_U8:
            case CVT_S8:
                _valaddr += 1;
                break;
            case CVT_U16:
            case CVT_S16:
                _valaddr += 2;
                break;
            case CVT_U32:
            case CVT_S32:
            case CVT_Float:
                _valaddr += 4;
                break;
            case CVT_U64:
            case CVT_S64:
            case CVT_Double:
                _valaddr += 8;
                break;
            default:
                break;
        }
    }
    // 写入参数
    memcpy((uint8_t*)d, &p->frame.data[3 + _valaddr], _vallen);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
以下参数和函数，是有关命令发送的，如果用户不需要单片机主动发送命令，则以下部分可删除
一般的，单片机均是执行命令的一侧，只需实现命令解析即可，只有单片机作为控制器，控制另一个设备时，才需要实现命令发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 命令发送相关定义
#define CMDSENDOVERTIME 1000
enum _eCmdSendStatus { Idle, WaitForCheck, CheckOK, CheckErr, CSSCount };
static uint8_t cmdSendStatus = Idle;
static uint8_t cmdInfoNeedToCheck[3] = {0};
static uint16_t cmdSendOverTimeCnt = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
发送命令检查
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t AnoPTv8CmdSendIsInIdle(void) {
    if (cmdSendStatus == Idle)
        return 1;
    else
        return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
发送命令
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t AnoPTv8CmdSend(const uint16_t linktype, const uint8_t daddr, const uint8_t* cmd, const uint16_t cmdlen) {
    if (cmdSendStatus != Idle) return 0;

    int bufindex = AnoPTv8GetFreeTxBufIndex();

    if (bufindex >= 0) {
        AnoPTv8TxBuf[bufindex].used = 1;

        AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8DEVID_MY;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
        AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xC0;
        /***********************************************************************/
        memcpy(AnoPTv8TxBuf[bufindex].dataBuf.frame.data, cmd, cmdlen);
        /***********************************************************************/
        AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = cmdlen;
        AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
        AnoPTv8TxBuf[bufindex].linkType = linktype;
        AnoPTv8TxBuf[bufindex].readyToSend = 1;
        cmdInfoNeedToCheck[0] = 0xC0;
        cmdInfoNeedToCheck[1] = AnoPTv8TxBuf[bufindex].dataBuf.frame.sumcheck;
        cmdInfoNeedToCheck[2] = AnoPTv8TxBuf[bufindex].dataBuf.frame.addcheck;
        cmdSendStatus = WaitForCheck;
        cmdSendOverTimeCnt = 0;
        return 1;
    } else
        return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
接收到命令校验
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8CmdRecvCheck(const uint16_t linktype, const _un_frame_v8* p) {
    if (p->frame.frameid == 0x00) {
        if (p->frame.data[0] == cmdInfoNeedToCheck[0] && p->frame.data[1] == cmdInfoNeedToCheck[1] && p->frame.data[2] == cmdInfoNeedToCheck[2]) {
            // 命令发送成功
            cmdSendStatus = CheckOK;
        } else {
            // 命令发送成功
            cmdSendStatus = CheckErr;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
命令发送功能运行线程
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8CmdRunThread1ms(void) {
    if (cmdSendStatus != Idle) {
        if (cmdSendOverTimeCnt < CMDSENDOVERTIME)
            cmdSendOverTimeCnt++;
        else {
            // 命令发送超时
            cmdSendOverTimeCnt = 0;
            cmdSendStatus = Idle;
        }
    } else if (cmdSendStatus == CheckOK) {
        // 命令发送成功
        cmdSendOverTimeCnt = 0;
        cmdSendStatus = Idle;
    } else if (cmdSendStatus == CheckErr) {
        // 命令发送失败
        cmdSendOverTimeCnt = 0;
        cmdSendStatus = Idle;
    }
}
