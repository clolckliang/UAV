#ifndef __ANOLINK_HPP__
#define __ANOLINK_HPP__

#include <chrono>

#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <thread>

#include "AnoPTv8.hpp"
#include "AnoPTv8ExAPI.hpp"
#include "anoSerial.hpp"
#include "anoUdp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/string.hpp"
#include <array>
#include <unordered_map>
#include "anolink_interfaces/srv/flight_command.hpp"
#include "anolink_interfaces/srv/set_flight_mode.hpp"
#include "anolink_interfaces/msg/flight_status.hpp"
#include "std_srvs/srv/trigger.hpp"

class AnoLink : public rclcpp::Node {
    public:
        struct sSettings {
            std::string serialName_FC;
            uint32_t serialBaud_FC;
            uint16_t udpPortUpPc;
            std::string pubTopicNameImu;
            std::string pubTopicNameMag;
            std::string pubTopicNameTmp;
            std::string pubTopicNameAlt;
            std::string pubTopicNameDDist;
            std::string pubTopicNameImuVel;
            std::string pubTopicNameOF;
            std::string pubTopicNameCmdVel;
            std::string subTopicNameCmdVel;
            std::string subTopicNamePos1;
            std::string subTopicNamePos2;
            std::string subTopicSlamPos;
            std::string subTopic_FC_Cmd;
        };

    public:
        AnoLink(std::string name);
        static AnoLink* m_self;
        sSettings m_set;
        AnoSerial* m_serialFC;
        AnoUdp* m_udpUpPc;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_pubImu;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_pubMag;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_pubTmp;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr m_pubAlt;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr m_pubDDist;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_pubImuVel;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_pubCmdVel;
        rclcpp::Publisher<anolink_interfaces::msg::FlightStatus>::SharedPtr m_pubFlightStatus;
        

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_unlockService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_lockService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_takeoffService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_landingService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_stableService;
        rclcpp::Service<anolink_interfaces::srv::SetFlightMode>::SharedPtr m_setFlightModeService;
        
        void handleStable(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void handleUnlock(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void handleLock(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void handleTakeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void handleLanding(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void handleSetFlightMode(const std::shared_ptr<anolink_interfaces::srv::SetFlightMode::Request> request,
                         std::shared_ptr<anolink_interfaces::srv::SetFlightMode::Response> response);


        void RxV8FrameCB(const uint8_t linktype, const _un_frame_v8* p);
    private:
        rclcpp::TimerBase::SharedPtr m_runTimer;

        tf2_ros::Buffer::SharedPtr m_tfBuf;
        std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr m_subPos;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subTwist;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub_FC_Cmd;

        std::thread* m_threadRun;

        void rosTimerCB1ms(void);
        void runThread(void);
        void tfListenerCheck(void);
};

#endif