# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

add_library(vesc-fw-interface INTERFACE)
target_include_directories(vesc-fw-interface SYSTEM INTERFACE ${CMAKE_CURRENT_LIST_DIR}/vesc)
