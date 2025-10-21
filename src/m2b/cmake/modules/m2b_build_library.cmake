# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

function(m2b_get_object_library_target_name name output)
    set(${output} ${name}-objects PARENT_SCOPE)
endfunction()

function(_m2b_get_project_object_libraries_property_name output)
    set(${output} M2B_PROJECT_${PROJECT_NAME}_OBJECTS PARENT_SCOPE)
endfunction()

function(_m2b_add_project_object_library name)
    m2b_get_object_library_target_name(${name} _target_name)
    _m2b_get_project_object_libraries_property_name(_property_name)
    set_property(GLOBAL APPEND PROPERTY ${_property_name} ${_target_name})
endfunction()

function(m2b_get_project_object_libraries output)
    _m2b_get_project_object_libraries_property_name(_property_name)
    get_property(_objects GLOBAL PROPERTY ${_property_name})
    set(${output} ${_objects} PARENT_SCOPE)
endfunction()

function(m2b_define_parent_target property_name)
    define_property(TARGET PROPERTY ${property_name} INHERITED
        BRIEF_DOCS "M2Build parent target property"
        FULL_DOCS "Defines which parent target this object library belongs into."
    )
endfunction()

function(m2b_define_selector property_name)
    define_property(TARGET PROPERTY ${property_name} INHERITED
        BRIEF_DOCS "M2Build selector property"
        FULL_DOCS "Defines what selectors this object library belongs into."
    )
endfunction()

function(m2b_set_selectors property_name selectors)
    set_property(DIRECTORY PROPERTY ${property_name} ${selectors})
endfunction()

function(m2b_get_target_object_libraries objects_output sources_output)
    cmake_parse_arguments(ARG "" "PARENT_TARGET;PARENT_TARGET_PROPERTY;SELECTOR_PROPERTY" "SELECTORS" ${ARGN})

    m2b_get_project_object_libraries(_proj_objs)

    set(_objects "")
    set(_sources "")

    foreach(_proj_obj IN LISTS _proj_objs)
        if (ARG_PARENT_TARGET AND ARG_PARENT_TARGET_PROPERTY)
            get_target_property(_parent_target ${_proj_obj} ${ARG_PARENT_TARGET_PROPERTY})
            if ("${_parent_target}" STREQUAL "${ARG_PARENT_TARGET}")
                list(APPEND _objects ${_proj_obj})
                list(APPEND _sources $<TARGET_OBJECTS:${_proj_obj}>)
            endif()
        endif()

        if (ARG_SELECTOR_PROPERTY AND ARG_SELECTORS)
            get_target_property(_obj_selectors ${_proj_obj} ${ARG_SELECTOR_PROPERTY})
            foreach(_obj_selector IN LISTS _obj_selectors)
                if ("${_obj_selector}" IN_LIST ARG_SELECTORS)
                    list(APPEND _objects ${_proj_obj})
                    list(APPEND _sources $<TARGET_OBJECTS:${_proj_obj}>)
                    break()
                endif()
            endforeach()
        endif()
    endforeach()

    set(${objects_output} ${_objects} PARENT_SCOPE)
    set(${sources_output} ${_sources} PARENT_SCOPE)
endfunction()

function(m2b_target_include_directories target_name)
    cmake_parse_arguments(ARG "" "" "FROM_PROPERTIES" ${ARGN})

    target_include_directories(${target_name} PRIVATE ${ARG_UNPARSED_ARGUMENTS})

    foreach(_property IN LISTS ARG_FROM_PROPERTIES)
        get_property(_dir GLOBAL PROPERTY ${_property})
        target_include_directories(${target_name} PRIVATE ${_dir})
    endforeach()
endfunction()

function(m2b_target_link_libraries target_name)
    cmake_parse_arguments(ARG "" "" "FROM_PROPERTIES" ${ARGN})

    target_link_libraries(${target_name} PRIVATE ${ARG_UNPARSED_ARGUMENTS})

    foreach(_property IN LISTS ARG_FROM_PROPERTIES)
        get_property(_library GLOBAL PROPERTY ${_property})
        target_link_libraries(${target_name} PRIVATE ${_library})
    endforeach()
endfunction()

function(m2b_add_object_library name)
    cmake_parse_arguments(ARG "" "" "INCLUDES;INCLUDES_FROM_PROPERTIES;DEFINES;DEPENDS;DEPENDS_FROM_PROPERTIES" ${ARGN})

    m2b_get_object_library_target_name(${name} _target_name)
    add_library(${_target_name} OBJECT ${ARG_UNPARSED_ARGUMENTS})

    # retain the original M2B_CLANG_TIDY_CHECK behaviour during transition
    if (M2B_CLANG_TIDY_CHECK AND NOT M2B_CLANG_TIDY_CHECK_C AND NOT M2B_CLANG_TIDY_CHECK_CXX)
        message(WARNING "M2B_CLANG_TIDY_CHECK is set. Defaulting to C++ only clang-tidy with header filter ${CMAKE_SOURCE_DIR}/code/include/.*h. This behaviour might cause error or change in the future.")
        set(M2B_CLANG_TIDY_CHECK_CXX_HEADER_FILTER "code/include/.*h$")
        set(M2B_CLANG_TIDY_CHECK_CXX ON)
    endif()

    # clang-tidy C check
    if (M2B_CLANG_TIDY_CHECK_C)
        if (NOT M2B_CLANG_TIDY_CHECK_C_HEADER_FILTER)
            message(FATAL "M2B_CLANG_TIDY_CHECK_C_HEADER_FILTER is not set.")
        endif()
        set(_c_clang_tidy_cmd "clang-tidy;--header-filter=${M2B_CLANG_TIDY_CHECK_C_HEADER_FILTER}")
        if (M2B_CLANG_TIDY_WARNINGS_AS_ERRORS)
            list(APPEND _c_clang_tidy_cmd -warnings-as-errors=*)
        endif()
        set_property(TARGET ${_target_name} PROPERTY C_CLANG_TIDY ${_c_clang_tidy_cmd})
        unset(_c_clang_tidy_cmd)
    endif()

    # clang-tidy C++ check
    if (M2B_CLANG_TIDY_CHECK_CXX)
        if (NOT M2B_CLANG_TIDY_CHECK_CXX_HEADER_FILTER)
            message(FATAL "M2B_CLANG_TIDY_CHECK_CXX_HEADER_FILTER is not set.")
        endif()
        set(_cxx_clang_tidy_cmd "clang-tidy;--header-filter=${M2B_CLANG_TIDY_CHECK_CXX_HEADER_FILTER}")
        if (M2B_CLANG_TIDY_WARNINGS_AS_ERRORS)
            list(APPEND _cxx_clang_tidy_cmd -warnings-as-errors=*)
        endif()
        set_property(TARGET ${_target_name} PROPERTY CXX_CLANG_TIDY ${_cxx_clang_tidy_cmd})
        unset(_cxx_clang_tidy_cmd)
    endif()
    
    if (ARG_INCLUDES OR ARG_INCLUDES_FROM_PROPERTIES)
        m2b_target_include_directories(${_target_name}
            ${ARG_INCLUDES}
            FROM_PROPERTIES ${ARG_INCLUDES_FROM_PROPERTIES})
    endif()

    if (ARG_DEFINES)
        target_compile_definitions(${_target_name} PRIVATE ${ARG_DEFINES})
    endif()

    if (ARG_DEPENDS OR ARG_DEPENDS_FROM_PROPERTIES)
        m2b_target_link_libraries(${_target_name}
            ${ARG_DEPENDS}
            FROM_PROPERTIES ${ARG_DEPENDS_FROM_PROPERTIES})
    endif()

    if (M2B_PLATFORM_STM32)
        set_property(TARGET ${_target_name} PROPERTY POSITION_INDEPENDENT_CODE OFF)
    else()
        set_property(TARGET ${_target_name} PROPERTY POSITION_INDEPENDENT_CODE ON)
    endif()

    _m2b_add_project_object_library(${name})
endfunction()

function(m2b_add_library target_name)
    cmake_parse_arguments(ARG "STATIC;SHARED" "PARENT_TARGET_PROPERTY;SELECTOR_PROPERTY" "WHOLE_DEPENDS;DEPENDS;SELECTORS;SRCS" ${ARGN})

    if (ARG_STATIC)
        set(_lib_type "STATIC")
    else()
        set(_lib_type "SHARED")
    endif()

    if (ARG_PARENT_TARGET_PROPERTY OR (ARG_SELECTOR_PROPERTY AND ARG_SELECTORS))
        m2b_get_target_object_libraries(_objs _srcs
            PARENT_TARGET ${target_name}
            PARENT_TARGET_PROPERTY ${ARG_PARENT_TARGET_PROPERTY}
            SELECTOR_PROPERTY ${ARG_SELECTOR_PROPERTY}
            SELECTORS ${ARG_SELECTORS}
        )
    endif()

    if (ARG_SRCS)
        list(APPEND _srcs ${ARG_SRCS})
    endif()

    if (NOT _srcs)
        message(FATAL_ERROR "No sources available for target ${target_name}. Please check whether the selectors or parent target are set correctly.")
    endif()

    add_library(${target_name} ${_lib_type} ${_srcs})

    if (ARG_WHOLE_DEPENDS)
        target_link_libraries(${target_name} PRIVATE -Wl,--whole-archive,--no-as-needed ${ARG_WHOLE_DEPENDS} -Wl,--no-whole-archive,--as-needed)
    endif()

    if (ARG_DEPENDS)
        target_link_libraries(${target_name} PRIVATE ${ARG_DEPENDS})
    endif()
endfunction()

function(m2b_add_interface name)
    cmake_parse_arguments(ARG "NO_INSTALL" "" "INCLUDES" ${ARGN})
    add_library(${name} INTERFACE)
    foreach(_include_dir IN LISTS ARG_INCLUDES)
        target_include_directories(${name} INTERFACE 
            $<BUILD_INTERFACE:${_include_dir}>
            $<INSTALL_INTERFACE:include>
        )
        if (NOT NO_INSTALL)
            install(DIRECTORY ${_include_dir}/ DESTINATION include/${PROJECT_NAME})
        endif()
    endforeach()
endfunction()

function(m2b_add_executable target_name)
    cmake_parse_arguments(ARG "" "PARENT_TARGET_PROPERTY;SELECTOR_PROPERTY" "WHOLE_DEPENDS;DEPENDS;SELECTORS;SRCS" ${ARGN})

    if (ARG_PARENT_TARGET_PROPERTY OR (ARG_SELECTOR_PROPERTY AND ARG_SELECTORS))
        m2b_get_target_object_libraries(_objs _srcs
            PARENT_TARGET ${target_name}
            PARENT_TARGET_PROPERTY ${ARG_PARENT_TARGET_PROPERTY}
            SELECTOR_PROPERTY ${ARG_SELECTOR_PROPERTY}
            SELECTORS ${ARG_SELECTORS}
        )
    endif()

    if (ARG_SRCS)
        list(APPEND _srcs ${ARG_SRCS})
    endif()

    add_executable(${target_name} ${_srcs})

    if (ARG_WHOLE_DEPENDS)
        target_link_libraries(${target_name} PRIVATE -Wl,--whole-archive,--no-as-needed ${ARG_WHOLE_DEPENDS} -Wl,--no-whole-archive,--as-needed)
    endif()

    if (ARG_DEPENDS)
        target_link_libraries(${target_name} PRIVATE ${ARG_DEPENDS})
    endif()
endfunction()

function(m2b_add_test_executable target_name)
    cmake_parse_arguments(ARG "" "PARENT_TARGET_PROPERTY" "DEPENDS" ${ARGN})
    m2b_add_executable(${target_name} ${ARG_UNPARSED_ARGUMENTS} PARENT_TARGET_PROPERTY ${ARG_PARENT_TARGET_PROPERTY} DEPENDS ${ARG_DEPENDS})

    if(M2B_USE_ROS_1)
        # do some black magic to make the test available in catkin
        add_dependencies(tests ${target_name})
        get_target_property(_target_path ${target_name} RUNTIME_OUTPUT_DIRECTORY)
        set(cmd "${_target_path}/${target_name} --gtest_output=xml:${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/gtest-${target_name}.xml")
        catkin_run_tests_target("gtest" ${target_name} "gtest-${target_name}.xml" COMMAND ${cmd} DEPENDENCIES ${target_name})
    endif()
endfunction()
