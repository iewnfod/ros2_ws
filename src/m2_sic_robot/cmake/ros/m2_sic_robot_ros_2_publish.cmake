m2_sic_robot_ros2_add_executable(fsm_node
    SELECTORS
        m2-sic-robot
        fsm-node
)

m2_sic_robot_ros2_add_executable(chassis_node
    SELECTORS
        m2-sic-robot
        chassis-node
)

m2b_install_targets(fsm_node)
m2b_install_targets(chassis_node)

m2b_ros2_python_package(${CMAKE_SOURCE_DIR}/code/scripts
    CONSOLE_SCRIPTS
        # ${CMAKE_SOURCE_DIR}/code/scripts/.template_node.py
)

m2b_ros2_cmake_package(
    EXPORT_DEPENDS
        rosidl_default_runtime
)
