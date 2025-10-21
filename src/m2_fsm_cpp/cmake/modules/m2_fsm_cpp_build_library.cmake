# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

find_package(trolly REQUIRED)
find_package(GTest REQUIRED)

function(m2_fsm_cpp_add_library target_name)
    cmake_parse_arguments(ARG "" "" "WHOLE_DEPENDS" ${ARGN})
    m2b_add_library(${target_name} SHARED ${ARG_UNPARSED_ARGUMENTS}
        SELECTOR_PROPERTY "M2_FSM_CPP_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            ${ARG_WHOLE_DEPENDS}
    )
endfunction()

function(m2_fsm_cpp_add_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
    m2b_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            trolly::trolly-interface
            m2-fsm-cpp-interface
            ${ARG_DEPENDS}
    )
endfunction()

function(m2_fsm_cpp_add_test_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
    m2b_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            trolly::trolly-interface
            trolly::trolly-test-interface
            m2-fsm-cpp-test-interface
            m2-fsm-cpp-interface
            ${ARG_DEPENDS}
    )
endfunction()

function(m2_fsm_cpp_add_executable target_name)
    m2b_add_executable(${target_name} ${ARGN}
        SELECTOR_PROPERTY "M2_FSM_CPP_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
            m2-fsm-cpp
    )
endfunction()

function(m2_fsm_cpp_add_test_executable target_name)
    m2b_add_test_executable(${target_name} ${ARGN}
        SELECTOR_PROPERTY "M2_FSM_CPP_SELECTOR"
        WHOLE_DEPENDS
            GTest::GTest
            trolly::trolly
            m2-fsm-cpp
            m2-fsm-cpp-unittest-main
    )
endfunction()

function(m2_fsm_cpp_build_current_dir object_name)
    aux_source_directory(. _srcs)
    m2_fsm_cpp_add_object_library(${object_name} ${_srcs})
endfunction()

function(m2_fsm_cpp_build_current_test_dir object_name)
    aux_source_directory(. _srcs)
    m2_fsm_cpp_add_test_object_library(${object_name} ${_srcs})
endfunction()
