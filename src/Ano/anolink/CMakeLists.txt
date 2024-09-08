cmake_minimum_required(VERSION 3.8)
project(anolink)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH /usr/local/lib)

include_directories(AnoPTv8 include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial REQUIRED)
find_package(anolink_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(anolink 
    src/anolink.cpp 
    src/anoSerial.cpp 
    src/anoUdp.cpp 
    src/anoSys.cpp 
    src/anoRos.cpp 
    AnoPTv8/AnoPTv8ExAPI.cpp
    AnoPTv8/AnoPTv8Cmd.cpp
    AnoPTv8/AnoPTv8Par.cpp
    AnoPTv8/AnoPTv8Run.cpp
    AnoPTv8/AnoPTv8FrameFactory.cpp
)

ament_target_dependencies(
    anolink 
    serial 
    rclcpp 
    tf2 
    tf2_ros 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    anolink_interfaces
    std_srvs
  )


install(TARGETS
  anolink
  DESTINATION lib/${PROJECT_NAME}
)

#添加yaml功能
find_package(yaml-cpp REQUIRED)
target_link_libraries(anolink yaml-cpp)

install(DIRECTORY config/   #src同目录的config，config带/的话，就是在目标目录创建config目录后再搬运，不带/就是直接搬运
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.yaml"
)

ament_package()
