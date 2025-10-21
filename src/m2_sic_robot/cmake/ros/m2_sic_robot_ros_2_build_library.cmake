find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(m2_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trolly REQUIRED)
find_package(m2_interfaces REQUIRED)
find_package(m2_fsm_cpp REQUIRED)

function(m2_sic_robot_ros2_add_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            trolly::trolly-interface
            m2_fsm_cpp::m2-fsm-cpp-interface
            m2-sic-robot-interface
            ${ARG_DEPENDS}
        AMENT_DEPENDS
            std_msgs
            std_srvs
            geometry_msgs
            nav_msgs
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_sic_robot_ros2_add_library target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_library(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "M2_SIC_ROBOT_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            trolly::trolly-ros2
            m2_fsm_cpp::m2-fsm-cpp
            m2_fsm_cpp::m2-fsm-cpp-ros2
            ${ARG_WHOLE_DEPENDS}
        AMENT_DEPENDS
            std_msgs
            std_srvs
            geometry_msgs
            nav_msgs
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_sic_robot_ros2_add_executable target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_executable(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "M2_SIC_ROBOT_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            trolly::trolly-ros2
            m2_fsm_cpp::m2-fsm-cpp
            m2_fsm_cpp::m2-fsm-cpp-ros2
            m2-sic-robot
            ${ARG_WHOLE_DEPENDS}
        AMENT_DEPENDS
            std_msgs
            std_srvs
            geometry_msgs
            nav_msgs
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_sic_robot_build_current_ros2_dir object_name)
    aux_source_directory(. _srcs)
    m2_sic_robot_ros2_add_object_library(${object_name} ${_srcs} ${ARGN})
endfunction()
