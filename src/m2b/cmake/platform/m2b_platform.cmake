# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

if(M2B_PLATFORM_WIN32)
    include(m2b_platform_win32)
elseif(M2B_PLATFORM_STM32)
    include(m2b_platform_stm32)
endif()
