cmake_minimum_required(VERSION 3.16)
project(m2_joy LANGUAGES CXX VERSION 1.0.0)
include(../m2b/cmake/modules/m2b_setup.cmake)

# Check if udev rules has been initialized
execute_process(
    COMMAND find /etc/udev/rules.d/51-m2-joy-setup.rules
    RESULT_VARIABLE res
)

if (res EQUAL 1)
    message(WARNING "udev rules has not been initialized, please run the setup script and build again. (Remove build/ install/)")
    return()
endif()

m2b_begin()
m2b_add_subdirectory(code)
m2b_end()