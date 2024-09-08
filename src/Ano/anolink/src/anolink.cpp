#include "anolink.hpp"

#include "anoSys.hpp"
#include "anoRos.hpp"
#include <rclcpp/logging.hpp>

AnoLink* AnoLink::m_self = nullptr;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnoLink>("anolink");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

AnoLink::AnoLink(std::string name) : Node(name) {
    if (m_self != nullptr) {
        return;
    }
    m_self = this;
    RCLCPP_INFO(this->get_logger(), "AnoLink init!");
    
    // 从配置文件读取配置
    readSettings();
    // 创建话题发布
    m_pubImu = this->create_publisher<sensor_msgs::msg::Imu>(m_set.pubTopicNameImu, rclcpp::SensorDataQoS());
    m_pubMag = this->create_publisher<sensor_msgs::msg::MagneticField>(m_set.pubTopicNameMag, rclcpp::SensorDataQoS());
    m_pubTmp = this->create_publisher<sensor_msgs::msg::Temperature>(m_set.pubTopicNameTmp, rclcpp::SensorDataQoS());
    m_pubAlt = this->create_publisher<sensor_msgs::msg::Range>(m_set.pubTopicNameAlt, rclcpp::SensorDataQoS());
    m_pubDDist = this->create_publisher<sensor_msgs::msg::Range>(m_set.pubTopicNameDDist, rclcpp::SensorDataQoS());
    m_pubImuVel = this->create_publisher<geometry_msgs::msg::TwistStamped>(m_set.pubTopicNameImuVel, rclcpp::SensorDataQoS());
    m_pubCmdVel = this->create_publisher<geometry_msgs::msg::TwistStamped>(m_set.pubTopicNameCmdVel, rclcpp::ParametersQoS());
    m_pubFlightStatus = this->create_publisher<anolink_interfaces::msg::FlightStatus>("flight_status", 10);
    // 创建位置话题tf监听
    m_tfBuf = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuf);
    // 创建速度话题监听
    //m_subTwist = this->create_subscription<geometry_msgs::msg::Twist>(m_set.subTopicNameCmdVel, rclcpp::SensorDataQoS(), std::bind(&AnoLink::rxCmdVel, this, std::placeholders::_1));
    m_subTwist = this->create_subscription<geometry_msgs::msg::Twist>(
    m_set.subTopicNameCmdVel, 
    rclcpp::SensorDataQoS(), 
    [=](const geometry_msgs::msg::Twist::SharedPtr msg)
     {
        rxMsgCmdVel(msg);
        });

    m_subPos=this->create_subscription<geometry_msgs::msg::Point>(
        m_set.subTopicSlamPos,
         rclcpp::SensorDataQoS(),
         [=](const geometry_msgs::msg::Point::SharedPtr msg){
            rxMsgPosVel(msg);
         });

    m_sub_FC_Cmd=this->create_subscription<std_msgs::msg::String>(
        m_set.subTopic_FC_Cmd,
        rclcpp::SensorDataQoS(),
        [=](const std_msgs::msg::String::SharedPtr cmd){
            rxMsg_FC_Cmd(cmd);
         });

    
    m_unlockService = this->create_service<std_srvs::srv::Trigger>(
        "unlock", 
        std::bind(
            &AnoLink::handleUnlock, 
            this, std::placeholders::_1, 
            std::placeholders::_2)
            );
        
    m_lockService = this->create_service<std_srvs::srv::Trigger>(
        "lock",
         std::bind(
            &AnoLink::handleLock, 
            this, std::placeholders::_1, 
            std::placeholders::_2)
            );

    m_takeoffService = this->create_service<std_srvs::srv::Trigger>(
        "takeoff", 
        std::bind(
            &AnoLink::handleTakeoff, 
            this, std::placeholders::_1, 
            std::placeholders::_2)
            );
    
    
    m_landingService = this->create_service<std_srvs::srv::Trigger>(
        "landing", 
        std::bind(
            &AnoLink::handleLanding, 
            this, std::placeholders::_1, 
            std::placeholders::_2)
            );

    m_stableService = this->create_service<std_srvs::srv::Trigger>(
        "Stable", 
        std::bind(
            &AnoLink::handleStable, 
            this, std::placeholders::_1, 
            std::placeholders::_2)
            );

    m_setFlightModeService = this->create_service<anolink_interfaces::srv::SetFlightMode>(
        "set_flight_mode", 
        std::bind(
            &AnoLink::handleSetFlightMode, 
            this, std::placeholders::_1, 
            std::placeholders::_2)
            );

    // 匿名协议初始化
    AnoPTv8ParInit();
    AnoPTv8CmdInit();
    // 通信连接初始化
    m_udpUpPc = new AnoUdp(m_set.udpPortUpPc, LT_UDP_UPPC, &AnoPTv8HwRecvByte);
    m_serialFC = new AnoSerial(m_set.serialName_FC, m_set.serialBaud_FC, LT_U1, &AnoPTv8HwRecvByte);
    // 新建线程
    m_threadRun = new std::thread(&AnoLink::runThread, this);
    // 创建运行timer
    m_runTimer = this->create_wall_timer(std::chrono::microseconds(1), std::bind(&AnoLink::rosTimerCB1ms, this));
}


void AnoLink::handleUnlock(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = sendFlightCommand("unlock");
    response->message = response->success ? "Unlock command sent successfully" : "Failed to send unlock command";
}

void AnoLink::handleLock(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = sendFlightCommand("lock");
    response->message = response->success ? "Lock command sent successfully" : "Failed to send lock command";
}

void AnoLink::handleTakeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = sendFlightCommand("take_off");
    response->message = response->success ? "Takeoff command sent successfully" : "Failed to send takeoff command";
}

void AnoLink::handleLanding(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = sendFlightCommand("landing");
    response->message = response->success ? "Landing command sent successfully" : "Failed to send landing command";
}

void AnoLink::handleStable(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = sendFlightCommand("Stable");
    response->message = response->success ? "Stable command sent successfully" : "Failed to send Stable command";
}

void AnoLink::handleSetFlightMode(const std::shared_ptr<anolink_interfaces::srv::SetFlightMode::Request> request,
                                  std::shared_ptr<anolink_interfaces::srv::SetFlightMode::Response> response) {
    if (request->mode == "stabilization_mode" || request->mode == "stabilization_height_mode" || 
        request->mode == "fixed_point_mode"|| request->mode == "fixed_point_and_program_control_mode") {
        response->success = sendFlightModeCommand(request->mode);
        response->message = response->success ? 
            "Flight mode set to " + request->mode : 
            "Failed to set flight mode to " + request->mode;
    } else {
        response->success = false;
        response->message = "Invalid flight mode requested. Use 'stabilization_mode' or 'stabilization_height_mode'"
                            "or 'fixed_point_mode' or 'fixed_point_and_program_control_mode'.";
    }
}


void AnoLink::rosTimerCB1ms(void) {
    tfListenerCheck();
}

void AnoLink::runThread(void) {
    while (true) {
        AnoPTv8HwTrigger1ms();
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}

void AnoLink::tfListenerCheck(void) {
    static std_msgs::msg::Header _lastHeader;
    geometry_msgs::msg::TransformStamped _tf;

    try {
        _tf = m_tfBuf->lookupTransform("toFrameRel", "fromFrameRel", tf2::TimePointZero);
        if (_tf.header.stamp != _lastHeader.stamp) {
            _lastHeader = _tf.header;
        }
    }
    catch (const tf2::TransformException& ex) {
        // RCLCPP_INFO(this->get_logger(), "Could not transform toFrameRel to
        // fromFrameRel: %s", ex.what());
        return;
    }
}

void AnoLink::RxV8FrameCB(const uint8_t linktype, const _un_frame_v8* p)
{
    switch (p->frame.frameid) {
    case 0x01://加速度、陀螺仪
    {
        if (LT_D_IMU != linktype)return;
        anoMsgImu.acc[0] = (float)*(int16_t*)(p->frame.data + 0) / 100;
        anoMsgImu.acc[1] = (float)*(int16_t*)(p->frame.data + 2) / 100;
        anoMsgImu.acc[2] = (float)*(int16_t*)(p->frame.data + 4) / 100;
        anoMsgImu.gyr[0] = (float)*(int16_t*)(p->frame.data + 6) / 16.384 * M_PI / 180;
        anoMsgImu.gyr[1] = (float)*(int16_t*)(p->frame.data + 8) / 16.384 * M_PI / 180;
        anoMsgImu.gyr[2] = (float)*(int16_t*)(p->frame.data + 10) / 16.384 * M_PI / 180;
        anoMsgImu.flagMemsReady = true;
        pubMsgImu();
    }
    break;

    case 0x02://罗盘、温度
    {
        anoMsgMag.x = (float)*(int16_t*)(p->frame.data + 0);
        anoMsgMag.y = (float)*(int16_t*)(p->frame.data + 2);
        anoMsgMag.z = (float)*(int16_t*)(p->frame.data + 4);
        anoMsgTmp = (float)*(int16_t*)(p->frame.data + 6) / 100;
        pubMsgMag();
        pubMsgTmp();
    }
    break;
    case 0x04://四元数
    {
        anoMsgImu.ori[0] = (float)*(int16_t*)(p->frame.data + 0) / 10000;
        anoMsgImu.ori[1] = (float)*(int16_t*)(p->frame.data + 2) / 10000;
        anoMsgImu.ori[2] = (float)*(int16_t*)(p->frame.data + 4) / 10000;
        anoMsgImu.ori[3] = (float)*(int16_t*)(p->frame.data + 6) / 10000;
        anoMsgImu.flagOriReady = true;
        pubMsgImu();
    }
    break;

    case 0x06: // 飞控运行模式
    {   aonStatus.current_flight_mode = p->frame.data[0];
        aonStatus.current_lock_state = p->frame.data[1];
        aonStatus.current_takeoff_state = p->frame.data[1];  // 假设起飞状态也在SFLAG中
        aonStatus.current_command_id = p->frame.data[2];
        aonStatus.current_command_0 = p->frame.data[3];
        aonStatus.current_command_1 = p->frame.data[4];
        pubFlightStatus();
    }
    break;

    case 0x0E:// 外接模块工作状态
    {                 
        aonStatus.current_velocity_sensor_state = p->frame.data[0];
        aonStatus.current_position_sensor_state = p->frame.data[1];
        aonStatus.current_gps_sensor_state = p->frame.data[2];
        aonStatus.current_altitude_sensor_state = p->frame.data[3];
        pubFlightStatus();  
    } 
    break;

    case 0x05://高度数据
    {
        anoMsgBar = (float)*(int32_t*)(p->frame.data + 0) / 100;
        anoMsgDDist = (float)*(int32_t*)(p->frame.data + 4) / 100;
        anoMsgAlt = (float)*(int32_t*)(p->frame.data + 8) / 100;
        pubMsgAlt();
    }
    break;
    case 0x07://imu速度
    {
        anoMsgImuSpd.x = (float)*(int16_t*)(p->frame.data + 0) / 100;
        anoMsgImuSpd.y = (float)*(int16_t*)(p->frame.data + 2) / 100;
        anoMsgImuSpd.z = (float)*(int16_t*)(p->frame.data + 4) / 100;
        pubMsgImuSpd();
    }
    break;
    case 0x51://匿名光流
    {
        if (p->frame.data[0] == 0) {
            //原始光流
            anoMsgOF.x = *(int8_t*)(p->frame.data + 2);
            anoMsgOF.y = *(int8_t*)(p->frame.data + 3);
            anoMsgOF.z = 0;
            if (0)
                pubMsgOF();
        }
        else if (p->frame.data[0] == 1) {
            //融合后
            anoMsgOF.x = (float)*(int16_t*)(p->frame.data + 2) / 100;
            anoMsgOF.y = (float)*(int16_t*)(p->frame.data + 4) / 100;
            anoMsgOF.z = 0;
            if (0)
                pubMsgOF();
        }
        else if (p->frame.data[0] == 2) {
            //惯导融合后
            if (1) {
                //融合后移动速度
                anoMsgOF.x = (float)*(int16_t*)(p->frame.data + 2) / 100;
                anoMsgOF.y = (float)*(int16_t*)(p->frame.data + 4) / 100;
                anoMsgOF.z = 0;
                if (1)
                    pubMsgOF();
            }
            else if (0) {
                //适合积分的融合后速度
                anoMsgOF.x = (float)*(int16_t*)(p->frame.data + 6) / 100;
                anoMsgOF.y = (float)*(int16_t*)(p->frame.data + 8) / 100;
                anoMsgOF.z = 0;
                if (0)
                    pubMsgOF();
            }
        }
    }
    break;
    }
}
