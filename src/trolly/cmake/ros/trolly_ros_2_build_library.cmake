# Copyright (c) 2025 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

function(trolly_ros2_add_object_library object_name)
    m2b_ros2_add_object_library(${object_name} ${ARGN}
        DEPENDS
            trolly-interface
    )
endfunction()

function(trolly_ros2_add_test_object_library object_name)
    m2b_ros2_add_object_library(${object_name} ${ARGN}
        INCLUDES_FROM_PROPERTIES
            TROLLY_GTEST_INCLUDES
        DEPENDS
            trolly-interface
            trolly-test-interface
            GTest::GTest
    )
endfunction()

function(trolly_ros2_add_library target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS" ${ARGN})
    m2b_ros2_add_library(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "TROLLY_SELECTOR"
        WHOLE_DEPENDS
            trolly
    )
endfunction()

function(trolly_build_current_ros2_dir object_name)
    aux_source_directory(. _srcs)
    trolly_ros2_add_object_library(${object_name} ${_srcs})
endfunction()

function(trolly_build_current_test_ros2_dir object_name)
    aux_source_directory(. _srcs)
    trolly_ros2_add_test_object_library(${object_name} ${_srcs})
endfunction()
