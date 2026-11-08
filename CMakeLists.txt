# 根 CMakeLists.txt — micro-ROS 智能小车 (/cmd_vel 键盘遥控 + PID 速度闭环)
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    ./bsp
    ./ros
    ./vision
    $ENV{HOME}/microros_ws/micro_ros_espidf_component
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(report_radar)
