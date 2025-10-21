# Copyright (c) 2023 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

include(CMakePackageConfigHelpers)

macro(m2b_init_project)
    list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
    list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/ros)

    include(${PROJECT_NAME}_build_library OPTIONAL)
    include(${PROJECT_NAME}_build_interface OPTIONAL)
    include(${PROJECT_NAME}_define_selectors OPTIONAL)

    if(M2B_USE_ROS_1)
        include(${PROJECT_NAME}_ros_1 OPTIONAL)
    endif()
    if(M2B_USE_ROS_2)
        include(${PROJECT_NAME}_ros_2_build_interface OPTIONAL)
        include(${PROJECT_NAME}_ros_2_build_library OPTIONAL)
    endif()
endmacro()

macro(m2b_begin)
    m2b_init_project()
endmacro()

macro(m2b_end)
    include(${PROJECT_NAME}_publish OPTIONAL)
    if(M2B_USE_ROS_2)
        include(${PROJECT_NAME}_ros_2_publish OPTIONAL)
    endif()
    m2b_export()
endmacro()

function(m2b_get_export_name output)
    set(${output} ${PROJECT_NAME}-export PARENT_SCOPE)
endfunction()

function(m2b_install_targets)
    m2b_get_export_name(_export_name)
    if (M2B_USE_ROS_2)
        # ROS 2 does not follow the FHS convention and everything has to be installed to lib folder to work
        # https://github.com/ros2/ros2cli/issues/845
        # To work properly with cmake packages and linking, we install exe to lib/${PROJECT_NAME} and library to lib only
        # Includes go into the include/${PROJECT_NAME} folder and it will be used in build only.
        install(TARGETS ${ARGN} EXPORT ${_export_name}
            RUNTIME DESTINATION lib/${PROJECT_NAME}
            LIBRARY DESTINATION lib
            INCLUDES DESTINATION include/${PROJECT_NAME}
        )
        return()
    endif()
    install(TARGETS ${ARGN} EXPORT ${_export_name}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include/${PROJECT_NAME}
    )
endfunction()

function(m2b_export)
    set(_dest_dir lib/${PROJECT_NAME}/cmake)

    m2b_get_export_name(_export_name)
    install(EXPORT ${_export_name} NAMESPACE ${PROJECT_NAME}:: DESTINATION ${_dest_dir})
    set(EXPORT_PATH ${_dest_dir}/${_export_name}.cmake)

    set(_config_path ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake)
    configure_package_config_file(${M2B_ROOT_DIR}/cmake/modules/m2b_config.cmake.in
      ${_config_path}
      INSTALL_DESTINATION ${_dest_dir}
      NO_SET_AND_CHECK_MACRO
      NO_CHECK_REQUIRED_COMPONENTS_MACRO
      PATH_VARS EXPORT_PATH
    )
    install(FILES ${_config_path} DESTINATION ${_dest_dir})
    
    set(_config_version_path ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake)
    write_basic_package_version_file(
        ${_config_version_path}
        COMPATIBILITY AnyNewerVersion
    )
    install(FILES ${_config_version_path} DESTINATION ${_dest_dir})
endfunction()
