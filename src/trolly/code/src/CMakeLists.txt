# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

trolly_set_selectors(standalone)

trolly_add_object_library(trolly-errno trolly_errno.cpp)

m2b_add_subdirectory(log)
m2b_add_subdirectory(math)

if (M2B_USE_ROS_2)
    m2b_add_subdirectory(ros2)
endif()
