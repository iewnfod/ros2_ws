# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

macro(m2b_ros1_package)
    cmake_parse_arguments(ARG "SETUP_PYTHON" "" "COMPONENTS;MESSAGES;SERVICES;GENERATE_MESSAGE_DEPENDS;CATKIN_DEPENDS;EXPORT_LIBRARIES;EXPORT_INCLUDE_DIRS" ${ARGN})

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_generation
        std_msgs
        ${ARG_COMPONENTS}
    )

    if(ARG_SETUP_PYTHON)
        catkin_python_setup()
    endif()

    if(ARG_MESSAGES)
        add_message_files(FILES ${ARG_MESSAGES})
    endif()

    if(ARG_SERVICES)
        add_service_files(FILES ${ARG_SERVICES})
    endif()

    if(ARG_MESSAGES OR ARG_SERVICES)
        generate_messages(DEPENDENCIES std_msgs ${ARG_GENERATE_MESSAGE_DEPENDS})
    endif()

    catkin_package(
        LIBRARIES ${ARG_EXPORT_LIBRARIES}
        INCLUDE_DIRS ${ARG_EXPORT_INCLUDE_DIRS}
        CATKIN_DEPENDS message_runtime ${ARG_CATKIN_DEPENDS}
    )

    set_property(GLOBAL PROPERTY M2B_ROS1_INCLUDES ${catkin_INCLUDE_DIRS})
    set_property(GLOBAL PROPERTY M2B_ROS1_LIBRARIES ${catkin_LIBRARIES})
endmacro()

function(m2b_ros1_add_object_library name)
    cmake_parse_arguments(ARG "" "" "INCLUDES_FROM_PROPERTIES;DEPENDS_FROM_PROPERTIES" ${ARGN})
    m2b_add_object_library(${name} ${ARG_UNPARSED_ARGUMENTS}
        INCLUDES_FROM_PROPERTIES M2B_ROS1_INCLUDES ${ARG_INCLUDES_FROM_PROPERTIES}
        DEPENDS_FROM_PROPERTIES M2B_ROS1_LIBRARIES ${ARG_DEPENDS_FROM_PROPERTIES}
    )
    # make exported targets like generate messages to be dependencies
    set(_exported_targets ${${PROJECT_NAME}_EXPORTED_TARGETS})
    if (_exported_targets)
        m2b_get_object_library_target_name(${name} _obj_lib_name)
        add_dependencies(${_obj_lib_name} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    endif()
endfunction()

function(m2b_ros1_add_library target_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
    get_property(_catkin_libs GLOBAL PROPERTY M2B_ROS1_LIBRARIES)
    m2b_add_library(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            ${_catkin_libs}
            ${ARG_DEPENDS})
endfunction()

function(m2b_ros1_add_executable target_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
    get_property(_catkin_libs GLOBAL PROPERTY M2B_ROS1_LIBRARIES)
    m2b_add_executable(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            ${_catkin_libs}
            ${ARG_DEPENDS})
endfunction()

function(m2b_ros1_add_test_executable target_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
    get_property(_catkin_libs GLOBAL PROPERTY M2B_ROS1_LIBRARIES)
    m2b_add_test_executable(${target_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            ${_catkin_libs}
            ${ARG_DEPENDS})
endfunction()
