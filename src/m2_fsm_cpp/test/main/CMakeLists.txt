# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

m2_fsm_cpp_set_selectors(unittest-main)

m2_fsm_cpp_build_current_test_dir(m2-fsm-cpp-unittest-main)

m2_fsm_cpp_add_library(m2-fsm-cpp-unittest-main
    SELECTORS
        unittest-main
)
