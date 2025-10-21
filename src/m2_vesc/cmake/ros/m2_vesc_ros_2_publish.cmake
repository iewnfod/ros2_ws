# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

m2_vesc_ros2_add_executable(vesc_node SELECTORS vesc-node)
m2b_ros2_install_targets(vesc_node)
m2b_ros2_cmake_package(
    EXPORT_DEPENDS
        rosidl_default_runtime
)
