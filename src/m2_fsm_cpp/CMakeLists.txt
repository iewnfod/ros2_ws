# Copyright (c) 2024 HKU Robocon Team
# Author: Eddie Wong (eddiewcw@connect.hku.hk)

cmake_minimum_required(VERSION 3.16)
project(m2_fsm_cpp LANGUAGES CXX VERSION 1.0.0)
include(../m2b/cmake/modules/m2b_setup.cmake)

m2b_begin()

m2b_add_subdirectory(code)
if (M2B_BUILD_TEST)
    m2b_add_subdirectory(test)
endif()

m2b_end()
