# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

option(M2B_BUILD_TEST "Enable build tests" ON)
option(M2B_BUILD_TSAN "Enable build tsan" OFF)
option(M2B_BUILD_ASAN "Enable build asan" OFF)
option(M2B_BUILD_UBSAN "Enable build asan" OFF)

if (M2B_BUILD_TEST)
    enable_testing()
endif()

if (M2B_BUILD_TSAN)
    message(STATUS "tsan enabled")
    add_compile_options(-fsanitize=thread)
    add_link_options(-fsanitize=thread)
endif()

if (M2B_BUILD_ASAN)
    message(STATUS "asan enabled")
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif()

if (M2B_BUILD_UBSAN)
    message(STATUS "ubsan enabled")
    add_compile_options(-fsanitize=undefined)
    add_link_options(-fsanitize=undefined)
endif()

# Enable more warnings
if (MSVC)
    # we do not treat warnings as errors in MSVC
    add_compile_options(/Wall /wd4820 /wd5039 /wd5045 /external:W0)
else()
    add_compile_options(-Wall -Wextra -Werror)
endif()
