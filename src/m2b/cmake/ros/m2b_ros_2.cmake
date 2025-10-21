# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

find_package(ament_cmake)
find_package(ament_cmake_python)
find_package(rclcpp)

function(_m2b_ros2_get_has_generated_interfaces output)
    get_property(_value GLOBAL PROPERTY _M2B_ROS2_HAS_GENERATED_INTERFACES)
    set(${output} ${_value} PARENT_SCOPE)
endfunction()

function(_m2b_ros2_set_has_generated_interfaces)
    set_property(GLOBAL PROPERTY _M2B_ROS2_HAS_GENERATED_INTERFACES True)
endfunction()

function(_m2b_ros2_get_generate_interfaces_target_name output)
    # for some ros2 linking reason, the target name has to be the project name
    set(${output} ${PROJECT_NAME} PARENT_SCOPE)
endfunction()

function(m2b_ros2_generate_interfaces)
    cmake_parse_arguments(ARG "" "" "INTERFACES;INTERFACE_DEPENDS" ${ARGN})
    _m2b_ros2_get_generate_interfaces_target_name(generate_interfaces_target_name)
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${generate_interfaces_target_name}
        ${ARG_INTERFACES}
        DEPENDENCIES ${ARG_INTERFACE_DEPENDS}
    )
    _m2b_ros2_set_has_generated_interfaces()
endfunction()

macro(m2b_ros2_cmake_package)
    cmake_parse_arguments(ARG "" "" "EXPORT_TARGETS;EXPORT_DEPENDS" ${ARGN})

    if (NOT ament_cmake_FOUND)
        message(FATAL "ament_cmake package is required to compile as a ROS2 CMake package")
    endif()

    if (ARG_EXPORT_TARGETS)
        foreach(_target IN LISTS ARG_EXPORT_TARGETS)
            ament_export_targets(${_target} HAS_LIBRARY_TARGET)
        endforeach()
    endif()

    if (ARG_EXPORT_DEPENDS)
        foreach(_dependency IN LISTS ARG_EXPORT_DEPENDS)
            ament_export_dependencies(${_dependency})
        endforeach()
    endif()

    ament_package(${ARG_UNPARSED_ARGUMENTS})
endmacro()

macro(m2b_ros2_python_package python_dir)
    cmake_parse_arguments(ARG "" "" "CONSOLE_SCRIPTS" ${ARGN})
    if (NOT ament_cmake_python_FOUND)
        message(FATAL "ament_cmake_python package is required to compile as a ROS2 CMake Python package")
    endif()
    ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR ${python_dir} ${ARG_UNPARSED_ARGUMENTS})
    foreach(_script IN LISTS ARG_CONSOLE_SCRIPTS)
        install(PROGRAMS ${_script} DESTINATION lib/${PROJECT_NAME})
    endforeach()
endmacro()

function(_m2b_ros2_link_ros_libraries target_name)
    cmake_parse_arguments(ARG "" "" "AMENT_DEPENDS" ${ARGN})
    ament_target_dependencies(${target_name} PUBLIC rclcpp ${ARG_AMENT_DEPENDS})

    # link against in-project generated interfaces
    _m2b_ros2_get_has_generated_interfaces(_has_generated_interfaces)
    if (${_has_generated_interfaces})
        find_package(rosidl_default_generators REQUIRED)
        _m2b_ros2_get_generate_interfaces_target_name(generate_interfaces_target_name)
        rosidl_get_typesupport_target(cpp_typesupport_target
            ${generate_interfaces_target_name} rosidl_typesupport_cpp)
        target_link_libraries(${target_name} PRIVATE "${cpp_typesupport_target}")
    endif()
endfunction()

function(m2b_ros2_add_object_library name)
    cmake_parse_arguments(ARG "" "" "AMENT_DEPENDS" ${ARGN})
    if (NOT rclcpp_FOUND)
        message(FATAL "rclcpp package is required to compile ROS2 C++ sources")
    endif()
    m2b_add_object_library(${name} ${ARG_UNPARSED_ARGUMENTS})

    # link against ros2 libraries
    m2b_get_object_library_target_name(${name} target_name)
    _m2b_ros2_link_ros_libraries(${target_name} AMENT_DEPENDS ${ARG_AMENT_DEPENDS})
endfunction()

function(m2b_ros2_add_library target_name)
    cmake_parse_arguments(ARG "" "" "AMENT_DEPENDS" ${ARGN})
    if (NOT rclcpp_FOUND)
        message(FATAL "rclcpp package is required to compile ROS2 C++ sources")
    endif()
    m2b_add_library(${target_name} ${ARG_UNPARSED_ARGUMENTS})
    _m2b_ros2_link_ros_libraries(${target_name} AMENT_DEPENDS ${ARG_AMENT_DEPENDS})
endfunction()

function(m2b_ros2_add_executable target_name)
    cmake_parse_arguments(ARG "" "" "AMENT_DEPENDS" ${ARGN})
    if (NOT rclcpp_FOUND)
        message(FATAL "rclcpp package is required to compile ROS2 C++ sources")
    endif()
    m2b_add_executable(${target_name} ${ARG_UNPARSED_ARGUMENTS})
    _m2b_ros2_link_ros_libraries(${target_name} AMENT_DEPENDS ${ARG_AMENT_DEPENDS})
endfunction()

# A wrapper function reserved for future purposes
function(m2b_ros2_add_test_executable target_name)
    cmake_parse_arguments(ARG "" "" "" ${ARGN})
    if (NOT rclcpp_FOUND)
        message(FATAL "rclcpp package is required to compile ROS2 C++ sources")
    endif()
    m2b_ros2_add_executable(${target_name} ${ARG_UNPARSED_ARGUMENTS})
endfunction()

# A wrapper function reserved for future purposes
function(m2b_ros2_install_targets)
    m2b_install_targets(${ARGN})
endfunction()
