# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

find_package(m2_interfaces REQUIRED)

function(m2_fsm_cpp_ros2_add_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            trolly::trolly-interface
            m2-fsm-cpp-interface
            ${ARG_DEPENDS}
        AMENT_DEPENDS
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_fsm_cpp_ros2_add_test_object_library object_name)
    cmake_parse_arguments(ARG "" "" "INCLUDES_FROM_PROPERTIES;DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        INCLUDES_FROM_PROPERTIES
            M2_FSM_CPP_GTEST_INCLUDES
            ${ARG_INCLUDES_FROM_PROPERTIES}
        DEPENDS
            trolly::trolly-interface
            m2-fsm-cpp-interface
            m2-fsm-cpp-test-interface
            GTest::GTest
            ${ARG_DEPENDS}
        AMENT_DEPENDS
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_fsm_cpp_ros2_add_library target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_library(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "M2_FSM_CPP_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            trolly::trolly-ros2
            ${ARG_WHOLE_DEPENDS}
        AMENT_DEPENDS
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_fsm_cpp_ros2_add_executable target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_executable(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "M2_FSM_CPP_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            trolly::trolly-ros2
            m2-fsm-cpp
            ${ARG_WHOLE_DEPENDS}
        AMENT_DEPENDS
            m2_interfaces
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_fsm_cpp_build_current_ros2_dir object_name)
    aux_source_directory(. _srcs)
    m2_fsm_cpp_ros2_add_object_library(${object_name} ${_srcs})
endfunction()

function(m2_fsm_cpp_build_current_test_ros2_dir object_name)
    aux_source_directory(. _srcs)
    m2_fsm_cpp_ros2_add_test_object_library(${object_name} ${_srcs})
endfunction()
