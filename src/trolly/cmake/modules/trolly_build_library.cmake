# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

if (M2B_BUILD_TEST)
    find_package(GTest REQUIRED)
endif()
set_property(GLOBAL PROPERTY TROLLY_GTEST_INCLUDES ${GTEST_INCLUDE_DIRS})

set(DEFINES)
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Trolly debug mode is ON")
    list(APPEND DEFINES TROLLY_DEBUG_MODE)
endif()

function(trolly_add_library target_name)
    if (TROLLY_STATIC)
        set(_link_type STATIC)
    else()
        set(_link_type SHARED)
    endif()
    m2b_add_library(${target_name} ${_link_type} ${ARGN}
        SELECTOR_PROPERTY "TROLLY_SELECTOR"
    )
endfunction()

function(trolly_add_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
    m2b_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            trolly-interface
            ${ARG_DEPENDS}
        DEFINES
            ${DEFINES}
    )
endfunction()

function(trolly_add_executable target_name)
    m2b_add_executable(${target_name} ${ARGN}
        SELECTOR_PROPERTY "TROLLY_SELECTOR"
    )
endfunction()

function(trolly_add_test_object_library object_name)
    cmake_parse_arguments(ARG "" "" "INCLUDES_FROM_PROPERTIES;DEPENDS" ${ARGN})
    trolly_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        INCLUDES_FROM_PROPERTIES
            TROLLY_GTEST_INCLUDES
            ${ARG_INCLUDES_FROM_PROPERTIES}
        DEPENDS
            trolly-test-interface
            GTest::GTest
            ${ARG_DEPENDS}
        DEFINES
            ${DEFINES}
    )
endfunction()

function(trolly_add_test_executable target_name)
    m2b_add_test_executable(${target_name} ${ARGN}
        SELECTOR_PROPERTY "TROLLY_SELECTOR"
    )
endfunction()

function(trolly_build_current_dir object_name)
    aux_source_directory(. _srcs)
    trolly_add_object_library(${object_name} ${_srcs} ${ARGN})
endfunction()

function(trolly_build_current_test_dir object_name)
    aux_source_directory(. _srcs)
    trolly_add_test_object_library(${object_name} ${_srcs} ${ARGN})
endfunction()
