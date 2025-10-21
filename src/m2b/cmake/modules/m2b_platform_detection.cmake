# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

# Make alias
set(M2B_PLATFORM_UNIX ${UNIX})
set(M2B_PLATFORM_APPLE ${APPLE})
set(M2B_PLATFORM_WIN32 ${WIN32})

# Linux Detection
if (UNIX AND NOT APPLE)
    set(M2B_PLATFORM_LINUX TRUE)
endif()

# STM32 Detection
if (TARGET stm32cubemx AND CMAKE_SYSTEM_PROCESSOR STREQUAL arm AND CMAKE_CROSSCOMPILING)
    set(M2B_PLATFORM_STM32 TRUE)
    set(M2B_PLATFORM_LINUX FALSE)
endif()

# Print Platform
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Detected Platform:")
if (M2B_PLATFORM_WIN32)
    message(STATUS "- Windows")
elseif (M2B_PLATFORM_APPLE)
    message(STATUS "- MacOS")
elseif (M2B_PLATFORM_LINUX)
    message(STATUS "- Linux")
elseif (M2B_PLATFORM_STM32)
    message(STATUS "- STM32")
endif()
