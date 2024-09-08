#include "anoSys.hpp"
#include "anoRos.hpp"
#include "anolink.hpp"

uint8_t testPar_u8 = 123;
int16_t testPar_s16 = -123;
char _strParTest[30] = "测试参数www.anotc.com";
const _st_par_info _parInfoList[] = {
    {&testPar_u8, PVT_U8, 0, 200, "testPar_u8", "测试参数，uint8"},
    {&testPar_s16, PVT_S16, -200, 200, "testPar_s16", "测试参数，int16"},
    {_strParTest, PVT_Str, 0, 30, "StrPar", "StrParForTest"},
};

void AnoPTv8ParInit(void) {
    // 根据定义的数据长度，除以信息结构体长度，确定参数数量
    int _c = sizeof(_parInfoList) / sizeof(_st_par_info);
    for (int i = 0; i < _c; i++) {
        // 循环调用参数注册函数，将参数信息注册至匿名V8协议解析器
        AnoPTv8ParRegister(&_parInfoList[i]);
    }
}
/***********************************************************************************************************************************************
************************************************************************************************************************************************
************************************************************************************************************************************************
************************************************************************************************************************************************
************************************************************************************************************************************************
***********************************************************************************************************************************************/
void cmdFunMove(const _un_frame_v8* p, uint16_t cmdindex) {
    float _par = 0;
    // 从命令中获取参数值
    AnoPTv8CmdValCpy(&_par, p, cmdindex, 0);
    // 打印这几个参数的值
    //AnoPTv8SendValStr(LT_UDP_UPPC, ANOPTV8DEVID_SWJ, _par, "_par");
    if (p->frame.data[0] == 0x01 && p->frame.data[1] == 0x01) {
        switch (p->frame.data[2]) {
        case 0x00:
            pubMsgCmdVel(0, 0, 0, 0, 0, 0);
            break;
        case 0x01:
            pubMsgCmdVel(_par, 0, 0, 0, 0, 0);
            break;
        case 0x02:
            pubMsgCmdVel(-_par, 0, 0, 0, 0, 0);
            break;
        case 0x03:
            pubMsgCmdVel(0, _par, 0, 0, 0, 0);
            break;
        case 0x04:
            pubMsgCmdVel(0, -_par, 0, 0, 0, 0);
            break;
        case 0x05:
            pubMsgCmdVel(0, 0, 0, 0, 0, _par * 3.1415926 / 180);
            break;
        case 0x06:
            pubMsgCmdVel(0, 0, 0, 0, 0, -_par * 3.1415926 / 180);
            break;
        }
    }

}
const _st_cmd_info _cmdInfoList[] = {
    // 3字节CMDID定义	+8字节CMDVAL定义	+命令名称 +命令介绍      +该命令的函数指针
    {{{0x01, 0x01, 0x00}, {CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA}}, "stop", "停止", cmdFunMove},
    {{{0x01, 0x01, 0x01}, {CVT_Float, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA}}, "move_f", "前进,m/s", cmdFunMove},
    {{{0x01, 0x01, 0x02}, {CVT_Float, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA}}, "move_b", "后退,m/s", cmdFunMove},
    {{{0x01, 0x01, 0x03}, {CVT_Float, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA}}, "move_l", "向左,m/s", cmdFunMove},
    {{{0x01, 0x01, 0x04}, {CVT_Float, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA}}, "move_r", "向右,m/s", cmdFunMove},
    {{{0x01, 0x01, 0x05}, {CVT_Float, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA}}, "turn_l", "左转,deg/s", cmdFunMove},
    {{{0x01, 0x01, 0x06}, {CVT_Float, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA, CVT_NA}}, "turn_r", "右转,deg/s", cmdFunMove},
};

void AnoPTv8CmdInit(void) {
    int CmdCount = sizeof(_cmdInfoList) / sizeof(_st_cmd_info);
    for (int i = 0; i < CmdCount; i++) AnoPTv8CmdRegister(&_cmdInfoList[i]);
}

void readSettings(void) {
    try {
        YAML::Node config = YAML::LoadFile("src/anolink/config/config.yaml");
        AnoLink::m_self->m_set.serialName_FC = getYamlSet_str(config, "SerialName_FC", "/dev/ttyUSB0");
        AnoLink::m_self->m_set.serialBaud_FC = getYamlSet_double(config, "SerialBaud_FC", 500000);
        AnoLink::m_self->m_set.udpPortUpPc = getYamlSet_double(config, "UdpPort_UpPc", 12333);
        AnoLink::m_self->m_set.pubTopicNameImu = getYamlSet_str(config, "PubTopicName_Imu", "ano_imu");
        AnoLink::m_self->m_set.pubTopicNameMag = getYamlSet_str(config, "PubTopicName_Mag", "ano_mag");
        AnoLink::m_self->m_set.pubTopicNameTmp = getYamlSet_str(config, "PubTopicName_Tmp", "ano_tmp");
        AnoLink::m_self->m_set.pubTopicNameAlt = getYamlSet_str(config, "PubTopicName_Alt", "ano_alt");
        AnoLink::m_self->m_set.pubTopicNameDDist = getYamlSet_str(config, "PubTopicName_DDist", "ano_ddist");
        AnoLink::m_self->m_set.pubTopicNameImuVel = getYamlSet_str(config, "PubTopicName_ImuVel", "ano_imuvel");
        AnoLink::m_self->m_set.pubTopicNameCmdVel = getYamlSet_str(config, "PubTopicName_CmdVel", "ano_cmdvel");
        AnoLink::m_self->m_set.pubTopicNameOF = getYamlSet_str(config, "PubTopicName_OF", "ano_of");
        AnoLink::m_self->m_set.subTopicNameCmdVel = getYamlSet_str(config, "SubTopicName_CmdVel", "turtle1/cmd_vel");
        AnoLink::m_self->m_set.subTopicSlamPos = getYamlSet_str(config, "SubTopicName_SlamPos", "/Pos");
        AnoLink::m_self->m_set.subTopic_FC_Cmd = getYamlSet_str(config, "SubTopicName_FC_Pos", "/FC_Cmd");
        RCLCPP_INFO(AnoLink::m_self->get_logger(), "settings read ok");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(AnoLink::m_self->get_logger(), "处理YAML文件时发生错误: %s", e.what());
    }
}

std::string getYamlSet_str(const YAML::Node& node, const std::string& key, const std::string _default) {
    try {
        return node[key].as<std::string>();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(AnoLink::m_self->get_logger(), "set:%s read err:%s", key.c_str(), e.what());
        return _default;
    }
}

double getYamlSet_double(const YAML::Node& node, const std::string& key, const double _default) {
    try {
        return node[key].as<double>();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(AnoLink::m_self->get_logger(), "set:%s read err:%s", key.c_str(), e.what());
        return _default;
    }
}

std::string toHex(const uint8_t* data, const uint16_t len) {
    std::stringstream _s;
    _s << std::uppercase << std::hex << std::setfill('0');
    for (int i = 0; i < len; i++) {
        _s << std::setw(2) << static_cast<unsigned>(data[i]);
    }
    return _s.str();
}
