# Copyright (c) 2025 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

trolly_ros2_add_library(trolly-ros2
    SELECTORS
        ros2
)

m2b_ros2_install_targets(trolly-ros2)

m2b_ros2_cmake_package()
