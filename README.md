

# Ano ROS 2 Package

## 项目简介

Ano 是一个 ROS 2 包，用于与匿名科创的飞控系统进行通信和控制。该包提供了与飞控硬件交互的接口，包括发送控制命令、接收传感器数据以及监控飞行状态。

## 功能特性

* 与飞控系统的串口通信
* UDP 网络通信支持
* 发布 IMU、磁力计、温度、高度等传感器数据
* 接收和处理飞行控制命令
* 飞行模式切换
* 实时状态监控和发布

## 依赖项

* ROS 2 (测试于 Foxy/Humble)
* anolink_interfaces (自定义消息和服务接口包)
* std_srvs
* sensor_msgs
* geometry_msgs
* tf2_ros

## 安装

1. 将此包克隆到您的 ROS 2 工作空间的 src 目录中：

```c
mkdir -p UAV/src

git clone https://github.com/clolckliang/UAV.git
```

2. 安装依赖：

   ```c
   cd ~/UAV
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. 编译工作空间：

   ```c
   colcon build 
   source install/setup.bash
   ```

## 使用方法

1. 启动 Ano 节点：

   ```c
   ros2 run anolink anolink
   ```

‍

2. 发送飞行命令：

   ```c
   ros2 service call /unlock std_srvs/srv/Trigger
   ros2 service call /takeoff std_srvs/srv/Trigger
   ```

3. 切换飞行模式：

   ```c
   ros2 service call /set_flight_mode anolink_interfaces/srv/SetFlightMode "{mode: 'stabilization_mode'}"
   ```

4. 发送速度控制命令：

   ```c
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

## 配置

配置文件位于 `config/config.yaml`​。您可以在此文件中修改串口设置、话题名称等参数。

## 主要话题

* ​`/ano_imu`​ (sensor_msgs/Imu): IMU 数据
* ​`/ano_mag`​ (sensor_msgs/MagneticField): 磁力计数据
* ​`/ano_alt`​ (sensor_msgs/Range): 高度数据
* ​`/flight_status`​ (anolink_interfaces/FlightStatus): 飞行状态

## 服务

* ​`/unlock`​ (std_srvs/Trigger): 解锁飞控
* ​`/lock`​ (std_srvs/Trigger): 飞控上锁
* ​`/takeoff`​ (std_srvs/Trigger): 起飞
* ​`/landing`​ (std_srvs/Trigger): 降落
* ​`/set_flight_mode`​ (anolink_interfaces/SetFlightMode): 设置飞行模式

## 贡献

欢迎提交问题报告和拉取请求。在提交拉取请求之前，请确保您的代码符合项目的编码规范。

## 许可证

[在此处添加您的许可证信息]

## 联系方式

[在此处添加您的联系信息]
