# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

trolly_add_interface()
add_library(trolly::trolly-interface ALIAS trolly-interface)
trolly_add_library(trolly
    SELECTORS
        standalone
)
add_library(trolly::trolly ALIAS trolly)

m2b_install_targets(trolly trolly-interface)

if (M2B_BUILD_TEST)
    m2b_install_targets(trolly-test-interface)
endif()
