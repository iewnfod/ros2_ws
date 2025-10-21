# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

m2_fsm_cpp_add_interface()

m2_fsm_cpp_add_library(m2-fsm-cpp
    SELECTORS
        standalone
)

m2b_install_targets(m2-fsm-cpp m2-fsm-cpp-interface)
