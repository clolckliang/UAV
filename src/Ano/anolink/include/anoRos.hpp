#ifndef __ANOROS_HPP__
#define __ANOROS_HPP__

#include "anolink.hpp"

struct sMsgImu {
    float acc[3];
    float gyr[3];
    float ori[4];
    bool flagMemsReady;
    bool flagOriReady;
};
struct sMsgMag {
    float x;
    float y;
    float z;
};
struct sMsgSpd {
    float x;
    float y;
    float z;
};
struct sMsgOF {
    float x;
    float y;
    float z;
};

struct sMsgStatus{

    uint8_t current_flight_mode;
    uint8_t current_lock_state;
    uint8_t current_takeoff_state;
    uint8_t current_command_id;
    uint8_t current_command_0;
    uint8_t current_command_1;
    uint8_t current_velocity_sensor_state;
    uint8_t current_position_sensor_state;
    uint8_t current_gps_sensor_state;
    uint8_t current_altitude_sensor_state;
};

enum class FlightMode : uint8_t {
    STABILIZATION = 0x00,
    STABILIZATION_HEIGHT = 0x01,
    FIXED_POINT = 0x02,
    FIXED_POINT_AND_PROGRAM = 0x03
};

// 使用unordered_map来查找命令对应的缓冲区
const std::unordered_map<std::string, FlightMode> MODE_MAP = {
    {"stabilization_mode", FlightMode::STABILIZATION},
    {"stabilization_height_mode", FlightMode::STABILIZATION_HEIGHT},
    {"fixed_point_mode", FlightMode::FIXED_POINT},
    {"fixed_point_and_program_control_mode", FlightMode::FIXED_POINT_AND_PROGRAM}
};

// 使用unordered_map来查找命令对应的缓冲区
static const std::unordered_map<std::string, std::vector<uint8_t>> command_map = {
    {"unlock", {0x10, 0x00, 0x01}},
    {"lock", {0x10, 0x00, 0x02}},
    {"take_off", {0x10, 0x00, 0x05, 0x96, 0}},
    {"landing", {0x10, 0x00, 0x06}},
    {"Stable", {0x10, 0x00, 0x06}}
};

extern sMsgStatus aonStatus;
extern sMsgImu anoMsgImu;
extern sMsgMag anoMsgMag;
extern sMsgSpd anoMsgImuSpd;
extern sMsgOF anoMsgOF;
extern float anoMsgTmp;
extern float anoMsgAlt;
extern float anoMsgBar;
extern float anoMsgDDist;

void pubMsgImu(void);

void pubMsgMag(void);

void pubMsgTmp(void);

void pubMsgAlt(void);

void pubMsgImuSpd(void);

void pubMsgOF(void);

void pubFlightStatus();

void rxMsgPosVel(const geometry_msgs::msg::Point::SharedPtr pos );

void rxMsgCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

void rxMsg_FC_Cmd(const std_msgs::msg::String::SharedPtr cmd);

void pubMsgCmdVel(const float lx, const float ly, const float lz, const float ax, const float ay, const float az);

bool trySendCommand(const std::vector<uint8_t>& buf, int maxRetries = 5);

bool sendFlightCommand(const std::string& command);

bool sendFlightModeCommand(const std::string& mode);

#endif