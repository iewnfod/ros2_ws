# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

m2_fsm_cpp_set_selectors(test)

m2b_add_subdirectory(base)
m2b_add_subdirectory(commands)

m2_fsm_cpp_add_test_executable(m2-fsm-cpp-test
    SELECTORS
        test
)

include(GoogleTest)
gtest_discover_tests(m2-fsm-cpp-test)
