# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

if(M2B_USE_ROS_1)
    include(m2b_ros_1)
endif()
if(M2B_USE_ROS_2)
    include(m2b_ros_2)
endif()

