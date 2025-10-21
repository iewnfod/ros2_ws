m2_joy_ros2_add_executable(joy_transform_data_node
    SELECTORS
        joy-transform-data-node
)
m2b_ros2_install_targets(joy_transform_data_node)

m2b_ros2_cmake_package(
    EXPORT_DEPENDS
        rosidl_default_runtime
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
