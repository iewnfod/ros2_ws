# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

# STN32 compile flag
if (STM32)
    target_compile_options(${STM32_TARGET} PRIVATE -O0 -Wall -g -gdwarf-2)
endif()
