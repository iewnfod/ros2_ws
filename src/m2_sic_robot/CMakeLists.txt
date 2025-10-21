cmake_minimum_required(VERSION 3.16)
project(m2_sic_robot LANGUAGES C CXX VERSION 1.0.0)
include(../m2b/cmake/modules/m2b_setup.cmake)

m2b_begin()

m2b_add_subdirectory(code)

m2b_end()
