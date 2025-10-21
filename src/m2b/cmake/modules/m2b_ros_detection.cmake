# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

# ROS Detection
# TODO: detect ros version, now only detects ROS1 (catkin)
if (DEFINED CATKIN_DEVEL_PREFIX)
    set(M2B_USE_ROS_1 TRUE)
elseif (DEFINED ENV{AMENT_PREFIX_PATH})
    set(M2B_USE_ROS_2 TRUE)
endif()

if(M2B_USE_ROS_1)
    message(STATUS "- Using ROS1 in build")
endif()

if(M2B_USE_ROS_2)
    message(STATUS "- Using ROS2 in build")
endif()
