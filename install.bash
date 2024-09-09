#!/bin/bash

# 更新系统
sudo apt-get update

# 提示用户选择是否安装 Rtab_map
read -p "Do you want to install Rtab_map? (y/n): " install_rtabmap
if [ "$install_rtabmap" == "y" ]; then
    echo "Installing Rtab_map..."
    sudo apt install -y ros-humble-rtabmap-ros
else
    echo "Skipping Rtab_map installation."
fi

# 提示用户选择是否安装 Serial
read -p "Do you want to install Serial? (y/n): " install_serial
if [ "$install_serial" == "y" ]; then
    echo "Installing Serial..."
    mkdir -p serial
    cd serial
    git clone https://github.com/ZhaoXiangBox/serial
    cd serial
    mkdir -p build
    cd build
    cmake ..
    make
    cd ../..
else
    echo "Skipping Serial installation."
fi

# 提示用户选择是否安装 Lidar 相关依赖
read -p "Do you want to install Lidar dependencies? (y/n): " install_lidar
if [ "$install_lidar" == "y" ]; then
    echo "Installing Lidar dependencies..."
    sudo apt install -y python3-colcon-common-extensions

    # 检测系统并安装相应的 diagnostic-updater
    if [ "$(lsb_release -sc)" == "focal" ]; then
        echo "Installing for Foxy..."
        sudo apt-get install -y ros-foxy-diagnostic-updater
    elif [ "$(lsb_release -sc)" == "bullseye" ]; then
        echo "Installing for Galactic..."
        sudo apt-get install -y ros-galactic-diagnostic-updater
    else
        echo "Installing for Humble..."
        sudo apt-get install -y ros-humble-diagnostic-updater
    fi

    # 安装 libpcap-dev
    echo "Installing libpcap-dev..."
    sudo apt-get install -y libpcap-dev
else
    echo "Skipping Lidar dependencies installation."
fi

# 提示用户选择是否安装 ROS2 Cartographer
read -p "Do you want to install ROS2 Cartographer? (y/n): " install_cartographer
if [ "$install_cartographer" == "y" ]; then
    echo "Installing ROS2 Cartographer..."
    sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros

    # 提示用户是否手动安装 Cartographer 及其依赖项
    read -p "Do you want to manually install Cartographer from source? (y/n): " manual_install_cartographer
    if [ "$manual_install_cartographer" == "y" ]; then
        echo "Installing Cartographer dependencies..."
        sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
        wstool init src
        cd src
        git clone https://ghproxy.com/https://github.com/ros2/cartographer.git
        git clone https://ghproxy.com/https://github.com/ros2/cartographer_ros.git
        wstool update -t src
        cd ..

        # 初始化 rosdep 并安装依赖
        echo "Initializing rosdep..."
        sudo rosdep init
        rosdep update
        rosdep install --from-paths src --ignore-src --rosdistro=humble -y

        # 编译 Cartographer
        echo "Building Cartographer..."
        colcon build
    else
        echo "Skipping manual Cartographer installation."
    fi
else
    echo "Skipping ROS2 Cartographer installation."
fi

echo "Installation process completed!"
