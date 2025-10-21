# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

find_package(trolly REQUIRED)
find_package(GTest REQUIRED)
set_property(GLOBAL PROPERTY M2_VESC_GTEST_INCLUDES ${GTEST_INCLUDE_DIRS})

function(m2_vesc_add_library target_name)
    m2b_add_library(${target_name} SHARED ${ARGN}
        SELECTOR_PROPERTY "M2_VESC_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
    )
endfunction()

function(m2_vesc_add_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
    m2b_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            m2-vesc-interface
            trolly::trolly-interface
            vesc-fw-interface
            ${ARG_DEPENDS}
    )
endfunction()

function(m2_vesc_add_executable target_name)
    m2b_add_executable(${target_name} ${ARGN}
        SELECTOR_PROPERTY "M2_VESC_SELECTOR"
    )
endfunction()

function(m2_vesc_add_test_object_library object_name)
    cmake_parse_arguments(ARG "" "" "INCLUDES_FROM_PROPERTIES;DEPENDS" ${ARGN})
    m2_vesc_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        INCLUDES_FROM_PROPERTIES
            M2_VESC_GTEST_INCLUDES
            ${ARG_INCLUDES_FROM_PROPERTIES}
        DEPENDS
            m2-vesc-interface
            m2-vesc-test-interface
            trolly::trolly-interface
            GTest::GTest
            ${ARG_DEPENDS}
    )
endfunction()

function(m2_vesc_add_test_executable target_name)
    m2b_add_test_executable(${target_name} ${ARGN}
        SELECTOR_PROPERTY "M2_VESC_SELECTOR"
    )
endfunction()

function(m2_vesc_build_current_dir object_name)
    aux_source_directory(. _srcs)
    m2_vesc_add_object_library(${object_name} ${_srcs} ${ARGN})
endfunction()

function(m2_vesc_build_current_test_dir object_name)
    aux_source_directory(. _srcs)
    m2_vesc_add_test_object_library(${object_name} ${_srcs} ${ARGN})
endfunction()
