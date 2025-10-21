# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

m2_fsm_cpp_ros2_add_library(m2-fsm-cpp-ros2
    SELECTORS
        standalone
        ros2
)
m2b_install_targets(m2-fsm-cpp-ros2)

m2b_ros2_cmake_package(
    EXPORT_DEPENDS
        rosidl_default_runtime
)
