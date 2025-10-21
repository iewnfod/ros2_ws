# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

cmake_minimum_required(VERSION 3.28)
project(m2_vesc LANGUAGES C CXX VERSION 1.0.0)
if (NOT M2B_ROOT_DIR)
    get_filename_component(M2B_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../m2b ABSOLUTE)
endif()
include(${M2B_ROOT_DIR}/cmake/modules/m2b_setup.cmake)

m2b_begin()

m2b_add_subdirectory(external)
m2b_add_subdirectory(code)
# if (M2B_BUILD_TEST)
#     enable_testing()
#     m2b_add_subdirectory(test)
# endif()

m2b_end()
