# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

function(trolly_add_interface)
    m2b_add_interface(trolly-interface
        INCLUDES ${PROJECT_SOURCE_DIR}/code/include
    )
endfunction()

function(trolly_add_test_interface)
    m2b_add_interface(trolly-test-interface
        INCLUDES ${PROJECT_SOURCE_DIR}/test/include
    )
endfunction()
