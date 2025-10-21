# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

function(m2_vesc_add_interface)
    m2b_add_interface(m2-vesc-interface
        INCLUDES ${PROJECT_SOURCE_DIR}/code/include
    )
endfunction()

function(m2_vesc_add_test_interface)
    m2b_add_interface(m2-vesc-test-interface
        INCLUDES ${PROJECT_SOURCE_DIR}/test/include
        NO_INSTALL
    )
endfunction()
