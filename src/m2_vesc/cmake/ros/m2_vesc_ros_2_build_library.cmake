# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

find_package(std_msgs REQUIRED)

function(m2_vesc_ros2_add_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_object_library(${object_name} ${ARGN}
        DEPENDS
            m2-vesc-interface
            trolly::trolly-interface
            ${ARG_DEPENDS}
        AMENT_DEPENDS
            std_msgs
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_vesc_ros2_add_test_object_library object_name)
    m2b_ros2_add_object_library(${object_name} ${ARGN}
        INCLUDES_FROM_PROPERTIES
            M2_VESC_GTEST_INCLUDES
        DEPENDS
            m2-vesc-interface
            m2-vesc-test-interface
            trolly::trolly-interface
            GTest::GTest
    )
endfunction()

function(m2_vesc_ros2_add_library target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_library(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "M2_VESC_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            trolly::trolly-ros2
            m2-vesc
            ${ARG_WHOLE_DEPENDS}
        AMENT_DEPENDS
            std_msgs
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_vesc_ros2_add_executable target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS;AMENT_DEPENDS" ${ARGN})
    m2b_ros2_add_executable(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "M2_VESC_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            trolly::trolly-ros2
            m2-vesc
            ${ARG_WHOLE_DEPENDS}
        AMENT_DEPENDS
            std_msgs
            ${ARG_AMENT_DEPENDS}
    )
endfunction()

function(m2_vesc_build_current_ros2_dir object_name)
    aux_source_directory(. _srcs)
    m2_vesc_ros2_add_object_library(${object_name} ${_srcs})
endfunction()

function(m2_vesc_build_current_test_ros2_dir object_name)
    aux_source_directory(. _srcs)
    m2_vesc_ros2_add_test_object_library(${object_name} ${_srcs})
endfunction()
