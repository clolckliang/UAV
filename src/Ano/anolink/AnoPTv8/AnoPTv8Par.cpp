#include "AnoPTv8Par.hpp"
#include "AnoPTv8ExAPI.hpp"
#include "AnoPTv8FrameFactory.hpp"
#define LIMIT(x, min, max) (((x) <= (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

// 定义一个指针数组，指针指向定义的参数信息结构体
const _st_par_info* pParInfoList[ANOPTV8_PAR_MAXCOUNT];
// 参数个数，每注册一个参数，个数+1
int parCount = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
V8协议解析器判断出来本帧数据为参数相关帧后，调用本函数进行进一步解析
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8ParFrameAnl(const uint16_t linktype, const _un_frame_v8* p) {
    switch (p->frame.frameid) {
        case 0xE0:
            // 参数命令
            if (p->frame.data[0] == 0) {
                // 读取设备信息
                AnoPTv8SendDevInfo(linktype, p->frame.sdevid);
            } else if (p->frame.data[0] == 1) {
                // 读取参数个数
                AnoPTv8SendParNum(linktype, p->frame.sdevid);
            } else if (p->frame.data[0] == 2) {
                // 读取参数值
                AnoPTv8SendParVal(linktype, p->frame.sdevid, *(uint16_t*)(p->frame.data + 1));
            } else if (p->frame.data[0] == 3) {
                // 读取参数信息
                AnoPTv8SendParInfo(linktype, p->frame.sdevid, *(uint16_t*)(p->frame.data + 1));
            } else {
                // 其他命令帧，需要返回check帧作为应答
                AnoPTv8SendCheck(linktype, p->frame.sdevid, p->frame.frameid, p->frame.sumcheck, p->frame.addcheck);
            }
            // 执行参数命令回调函数，用于处理其他参数命令
            AnoPTv8HwParCmdRecvCallback(linktype, p);
            break;
        case 0xE1:
            // 参数写入
            anoPTv8ParSetVal(p);
            // 返回check帧作为应答
            AnoPTv8SendCheck(linktype, p->frame.sdevid, p->frame.frameid, p->frame.sumcheck, p->frame.addcheck);
            // 执行参数值回调函数
            AnoPTv8HwParValRecvCallback(linktype, p);
            break;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
获取已注册的参数个数
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int AnoPTv8ParGetCount(void) {
    return parCount;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
通过指向参数的指针是否存在来判断该参数是否已经注册过
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t AnoPTv8ParCheckExist(const void* p) {
    for (int i = 0; i < AnoPTv8ParGetCount(); i++) {
        if (pParInfoList[i]->pval == p) return 1;
    }
    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
参数注册函数
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8ParRegister(const _st_par_info* _pi) {
    // 首先判断参数是否已经注册，通过指向参数的指针是否已经存在来判断
    if (!AnoPTv8ParCheckExist(_pi->pval)) {
        // 将参数信息写入参数列表，完成注册
        if (parCount <= (ANOPTV8_PAR_MAXCOUNT - 1)) {
            pParInfoList[parCount++] = _pi;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
根据参数index获取参数信息
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const _st_par_info* AnoPTv8ParGetInfo(const uint16_t parid) {
    if (parid > AnoPTv8ParGetCount()) return 0;
    return pParInfoList[parid];
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
将参数值写入指针d指向的地址，用于参数值发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t AnoPTv8ParCpyVal(const uint16_t parid, uint8_t* d) {
    if (parid > AnoPTv8ParGetCount()) return 0;
    switch (pParInfoList[parid]->type) {
        case PVT_U8:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 1);
            return 1;
        case PVT_S8:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 1);
            return 1;
        case PVT_U16:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 2);
            return 2;
        case PVT_S16:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 2);
            return 2;
        case PVT_U32:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 4);
            return 4;
        case PVT_S32:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 4);
            return 4;
        case PVT_Float:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 4);
            return 4;
        case PVT_U64:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 8);
            return 8;
        case PVT_S64:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 8);
            return 8;
        case PVT_Double:
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), 8);
            return 8;
        case PVT_Str: {
            uint8_t _strlen = 0;
            while (_strlen < 100 && *((uint8_t*)(pParInfoList[parid]->pval) + _strlen) != 0) _strlen++;
            memcpy(d, (uint8_t*)(pParInfoList[parid]->pval), _strlen);
            return _strlen;
        }
        default:
            return 0;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
接收到参数值以后，根据参数注册时的地址指针，将参数值写入该地址
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void anoPTv8ParSetVal(const _un_frame_v8* p) {
    uint16_t parid = *(uint16_t*)(p->frame.data);
    if (parid > AnoPTv8ParGetCount()) return;
    switch (pParInfoList[parid]->type) {
        case PVT_U8: {
            uint8_t _v = *(uint8_t*)(p->frame.data + 2);
            *(uint8_t*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_S8: {
            int8_t _v = *(int8_t*)(p->frame.data + 2);
            *(int8_t*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_U16: {
            uint16_t _v = *(uint16_t*)(p->frame.data + 2);
            *(uint16_t*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_S16: {
            int16_t _v = *(int16_t*)(p->frame.data + 2);
            *(int16_t*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_U32: {
            uint32_t _v = *(uint32_t*)(p->frame.data + 2);
            *(uint32_t*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_S32: {
            int32_t _v = *(int32_t*)(p->frame.data + 2);
            *(int32_t*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_Float: {
            float _v = *(float*)(p->frame.data + 2);
            *(float*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_U64: {
            uint64_t _v = *(uint64_t*)(p->frame.data + 2);
            *(uint64_t*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_S64: {
            int64_t _v = *(int64_t*)(p->frame.data + 2);
            *(int64_t*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_Double: {
            double _v = *(double*)(p->frame.data + 2);
            *(double*)(pParInfoList[parid]->pval) = LIMIT(_v, pParInfoList[parid]->min, pParInfoList[parid]->max);
        } break;
        case PVT_Str: {
            uint8_t _strlen = p->frame.datalen - 2;
            if (_strlen > 99) _strlen = 99;
            memset(pParInfoList[parid]->pval, 0, pParInfoList[parid]->max);
            memcpy((uint8_t*)(pParInfoList[parid]->pval), (uint8_t*)(p->frame.data + 2), _strlen);
        } break;
        default:
            break;
    }
}
