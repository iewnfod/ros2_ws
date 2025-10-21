find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(m2_interfaces REQUIRED)

function(m2_joy_ros2_add_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            trolly::trolly-interface
            m2-joy-interface
            ${ARG_DEPENDS}
        AMENT_DEPENDS
            std_msgs
            sensor_msgs
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_joy_ros2_add_library target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_library(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "M2_CHASSIS_KINEMATICS_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            trolly::trolly-ros2
            ${ARG_WHOLE_DEPENDS}
        AMENT_DEPENDS
            std_msgs
            sensor_msgs
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_joy_ros2_add_executable target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_executable(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "M2_JOY_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            trolly::trolly-ros2
            ${ARG_WHOLE_DEPENDS}
        AMENT_DEPENDS
            std_msgs
            sensor_msgs
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_joy_build_current_ros2_dir object_name)
    aux_source_directory(. _srcs)
    m2_joy_ros2_add_object_library(${object_name} ${_srcs})
endfunction()

function(m2_joy_build_current_test_ros2_dir object_name)
    aux_source_directory(. _srcs)
    m2_joy_ros2_add_test_object_library(${object_name} ${_srcs})
endfunction()
