# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

function(m2b_add_subdirectory dir)
    add_subdirectory(${dir} ${ARGN})
endfunction()

function(m2b_submodule_clone)
    cmake_parse_arguments(ARG "" "" "VERIFY_PATHS" ${ARGN})

    find_package(Git QUIET)
    if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    # Update submodules as needed
        option(GIT_SUBMODULE "Check submodules during build" ON)
        if(GIT_SUBMODULE)
            message(STATUS "Submodule update")
            execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                            RESULT_VARIABLE GIT_SUBMOD_RESULT)
            if(NOT GIT_SUBMOD_RESULT EQUAL "0")
                message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
            endif()
        endif()
    endif()

    foreach(_path IN LISTS ARG_VERIFY_PATHS)
        if(NOT EXISTS ${_path})
            message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
        endif()
    endforeach()
endfunction()

function(m2b_target_link_threads target_name)
    # Link the pthread library if in UNIX-like systems
    if (UNIX)
        set(THREADS_PREFER_PTHREAD_FLAG ON)
        find_package(Threads REQUIRED)
        target_link_libraries(${target_name} PRIVATE Threads::Threads)
    endif()
endfunction()

function(m2b_target_link_maths target_name)
    # Link the pthread library if in UNIX-like systems
    if (UNIX)
        # Link the math library if not MSVC
        if (NOT MSVC)
            target_link_libraries(${target_name} PRIVATE m)
        endif()
    endif()
endfunction()
