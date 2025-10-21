# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

trolly_set_selectors(test)

trolly_build_current_test_dir(test)
m2b_add_subdirectory(log)
m2b_add_subdirectory(math)
if (M2B_USE_ROS_2)
    m2b_add_subdirectory(ros2)
endif()

trolly_add_test_executable(trolly-test
    SELECTORS
        test
    DEPENDS
        trolly
        trolly-unittest-main
        GTest::GTest
)

include(GoogleTest)
gtest_discover_tests(trolly-test)
