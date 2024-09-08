#include "anoRos.hpp"
#include "AnoPTv8Cmd.hpp"

sMsgStatus aonStatus;
sMsgImu anoMsgImu;
sMsgMag anoMsgMag;
sMsgSpd anoMsgImuSpd;
sMsgOF anoMsgOF;
float anoMsgTmp;
float anoMsgAlt;
float anoMsgBar;
float anoMsgDDist;



void pubMsgImu(void) {
    if (anoMsgImu.flagMemsReady && anoMsgImu.flagOriReady) {
        anoMsgImu.flagMemsReady = false;
        anoMsgImu.flagOriReady = false;

        auto message = sensor_msgs::msg::Imu();
        message.header.stamp = AnoLink::m_self->get_clock()->now();
        message.header.frame_id = "imu_link";

        // Fill in IMU data
        message.orientation.w = anoMsgImu.ori[0];
        message.orientation.x = anoMsgImu.ori[1];
        message.orientation.y = anoMsgImu.ori[2];
        message.orientation.z = anoMsgImu.ori[3];

        message.angular_velocity.x = anoMsgImu.gyr[0];  // Radians/sec
        message.angular_velocity.y = anoMsgImu.gyr[1];  // Radians/sec
        message.angular_velocity.z = anoMsgImu.gyr[2];  // Radians/sec

        message.linear_acceleration.x = anoMsgImu.acc[0];  // m/s^2
        message.linear_acceleration.y = anoMsgImu.acc[1];  // m/s^2
        message.linear_acceleration.z = anoMsgImu.acc[2];  // m/s^2

        AnoLink::m_self->m_pubImu->publish(message);
        RCLCPP_INFO(AnoLink::m_self->get_logger(), "iiimmmuuu");
    }
}

void pubFlightStatus(void) {
    auto msg = anolink_interfaces::msg::FlightStatus();
    
    // 假设这些值是从某处获取的
    msg.flight_mode = aonStatus.current_flight_mode;
    msg.lock_state = aonStatus.current_lock_state;
    msg.takeoff_state = aonStatus.current_takeoff_state;
    msg.command_id = aonStatus.current_command_id;
    msg.command_0 = aonStatus.current_command_0;
    msg.command_1 = aonStatus.current_command_1;
    msg.velocity_sensor_state = aonStatus.current_velocity_sensor_state;
    msg.position_sensor_state = aonStatus.current_position_sensor_state;
    msg.gps_sensor_state = aonStatus.current_gps_sensor_state;
    msg.altitude_sensor_state = aonStatus.current_altitude_sensor_state;

    AnoLink::m_self->m_pubFlightStatus->publish(msg);
}

void pubMsgMag(void) {
    auto message = sensor_msgs::msg::MagneticField();
    message.header.stamp = AnoLink::m_self->get_clock()->now();
    message.header.frame_id = "imu_link";

    message.magnetic_field.x = anoMsgMag.x;
    message.magnetic_field.y = anoMsgMag.y;
    message.magnetic_field.z = anoMsgMag.z;

    AnoLink::m_self->m_pubMag->publish(message);
}

void pubMsgTmp(void) {
    auto message = sensor_msgs::msg::Temperature();
    message.header.stamp = AnoLink::m_self->get_clock()->now();
    message.header.frame_id = "imu_link";

    message.temperature = anoMsgTmp;

    AnoLink::m_self->m_pubTmp->publish(message);
}

void pubMsgAlt(void) {
    auto message = sensor_msgs::msg::Range();
    message.header.stamp = AnoLink::m_self->get_clock()->now();
    message.header.frame_id = "imu_link";
    message.radiation_type = 0;
    message.min_range = 0;
    message.max_range = 99999;
    message.range = anoMsgAlt;
    AnoLink::m_self->m_pubAlt->publish(message);

    message.radiation_type = 1;
    message.min_range = 0.1;
    message.max_range = 10;
    message.range = anoMsgDDist;
    AnoLink::m_self->m_pubDDist->publish(message);
}

void pubMsgImuSpd(void) {
    auto message = geometry_msgs::msg::TwistStamped();
    message.header.stamp = AnoLink::m_self->get_clock()->now();
    message.header.frame_id = "imu_link";
    message.twist.linear.x = anoMsgImuSpd.x;
    message.twist.linear.y = anoMsgImuSpd.y;
    message.twist.linear.z = anoMsgImuSpd.z;
    AnoLink::m_self->m_pubImuVel->publish(message);
}

void pubMsgOF(void) {
    auto message = geometry_msgs::msg::TwistStamped();
    message.header.stamp = AnoLink::m_self->get_clock()->now();
    message.header.frame_id = "imu_link";
    message.twist.linear.x = anoMsgOF.x;
    message.twist.linear.y = anoMsgOF.y;
    message.twist.linear.z = anoMsgOF.z;
    AnoLink::m_self->m_pubImuVel->publish(message);
}

void rxMsgPosVel(const geometry_msgs::msg::Point::SharedPtr pos ){
    uint8_t _buf[12];
    uint8_t _cnt = 0;
    
    // 将 double 类型的 x 和 y 转换为 int32_t
    int32_t _val_x = static_cast<int32_t>(pos->x * 1000);  // 根据需求放大或缩小，以保持精度
    memcpy(_buf + _cnt, &_val_x, 4);
    _cnt += 4;

    int32_t _val_y = static_cast<int32_t>(pos->y * 1000);  // 根据需求放大或缩小，以保持精度
    memcpy(_buf + _cnt, &_val_y, 4);
    _cnt += 4;

    int32_t _val_z = 0;  // z 方向默认为0
    memcpy(_buf + _cnt, &_val_z, 4);
    _cnt += 4;

    // 发送数据缓冲区
    AnoPTv8SendBuf(0xFF, 0xFF, 0x32, _buf, sizeof(_buf));
}


void rxMsgCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    uint8_t _buf[14];
    uint8_t _cnt = 0;
    int16_t _val = 0;
    memcpy(_buf + _cnt, (uint8_t*)(&_val), 2);
    _cnt += 2;
    _val = 0;
    memcpy(_buf + _cnt, (uint8_t*)(&_val), 2);
    _cnt += 2;
    _val = 0;
    memcpy(_buf + _cnt, (uint8_t*)(&_val), 2);
    _cnt += 2;
    _val = msg->angular.z * 180 / 3.1415926;
    memcpy(_buf + _cnt, (uint8_t*)(&_val), 2);
    _cnt += 2;
    _val = msg->linear.x * 100;
    memcpy(_buf + _cnt, (uint8_t*)(&_val), 2);
    _cnt += 2;
    _val = msg->linear.y * 100;
    memcpy(_buf + _cnt, (uint8_t*)(&_val), 2);
    _cnt += 2;
    _val = msg->linear.z * 100;
    memcpy(_buf + _cnt, (uint8_t*)(&_val), 2);

    // RCLCPP_INFO(this->get_logger(), "cmdvel_lx: %f", msg->linear.x);
    // RCLCPP_INFO(this->get_logger(), "cmdvel_ly: %f", msg->linear.y);
    // RCLCPP_INFO(this->get_logger(), "cmdvel_lz: %f", msg->linear.z);
    // RCLCPP_INFO(this->get_logger(), "cmdvel_ax: %f", msg->angular.x);
    // RCLCPP_INFO(this->get_logger(), "cmdvel_ay: %f", msg->angular.y);
    // RCLCPP_INFO(this->get_logger(), "cmdvel_az: %f", msg->angular.z);

    AnoPTv8SendBuf(0xFF, 0xFF, 0x41, _buf, sizeof(_buf));
}

void rxMsg_FC_Cmd(const std_msgs::msg::String::SharedPtr cmd)
{

    // 查找命令对应的缓冲区数据
    auto it = command_map.find(cmd->data);
    if (it != command_map.end())
    {
        // 找到命令，准备并发送相应的数据
        const auto &buf = it->second;
        if (AnoPTv8CmdSendIsInIdle()) {
            AnoPTv8CmdSend(0xFF, 0xFF, buf.data(), buf.size());
            RCLCPP_INFO(rclcpp::get_logger("FlightControlNode"), "Command '%s' received and sent to flight control.", cmd->data.c_str());
        // 日志信息：命令已发送（替换为你自己的日志实现）
        }
       
    }
    else
    {
        // 未知命令，输出警告信息（替换为你自己的日志实现）
        RCLCPP_WARN(rclcpp::get_logger("FlightControlNode"), "Received unknown command: %s", cmd->data.c_str());
    }
}

void pubMsgCmdVel(const float lx, const float ly, const float lz, const float ax, const float ay, const float az) {
    auto _msg = geometry_msgs::msg::TwistStamped();
    _msg.header.stamp = AnoLink::m_self->get_clock()->now();
    _msg.header.frame_id = "imu_link";
    _msg.twist.linear.x = lx;
    _msg.twist.linear.y = ly;
    _msg.twist.linear.z = lz;
    _msg.twist.angular.x = ax;
    _msg.twist.angular.y = ay;
    _msg.twist.angular.z = az;
    AnoLink::m_self->m_pubCmdVel->publish(_msg);
}

bool trySendCommand(const std::vector<uint8_t>& buf, int maxRetries) {
    for (int i = 0; i < maxRetries; ++i) {
        if (AnoPTv8CmdSendIsInIdle()) {
            AnoPTv8CmdSend(0xFF, 0xFF, buf.data(), buf.size());
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

bool sendFlightCommand(const std::string& command) {
    auto it = command_map.find(command);
    if (it != command_map.end()) {
        const auto& buf = it->second;       
        if (trySendCommand(buf)) {
            RCLCPP_INFO(AnoLink::m_self->get_logger(), "Command '%s' sent to flight control.", command.c_str());
            return true;
        } else {
            RCLCPP_WARN(AnoLink::m_self->get_logger(), "Failed to send command '%s' after multiple attempts.", command.c_str());
            return false;
        }
    } else {
        RCLCPP_WARN(AnoLink::m_self->get_logger(), "Received unknown command: %s", command.c_str());
        return false;
    }
}



bool sendFlightModeCommand(const std::string& mode) {
    auto it = MODE_MAP.find(mode);
    if (it == MODE_MAP.end()) {
        RCLCPP_WARN(AnoLink::m_self->get_logger(), "Invalid flight mode: %s", mode.c_str());
        return false;
    }

    std::vector<uint8_t> buf = {0x01, 0x01, 0x01, static_cast<uint8_t>(it->second)};
    if (trySendCommand(buf)) {
        RCLCPP_INFO(AnoLink::m_self->get_logger(), "Flight mode set to %s", mode.c_str());
        return true;
    }
    else {
        RCLCPP_WARN(AnoLink::m_self->get_logger(), "Failed to set flight mode to %s after multiple attempts", mode.c_str());
        return false;
    }
}