include_guard(GLOBAL)

find_package(trolly REQUIRED)
find_package(m2_fsm_cpp REQUIRED)

function(m2_sic_robot_add_library target_name)
    m2b_add_library(${target_name} SHARED ${ARGN}
        SELECTOR_PROPERTY "M2_SIC_ROBOT_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
    )
endfunction()

function(m2_sic_robot_add_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
    m2b_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            trolly::trolly-interface
            m2_fsm_cpp::m2-fsm-cpp-interface
            m2-sic-robot-interface
            ${ARG_DEPENDS}
    )
endfunction()

function(m2_sic_robot_add_executable target_name)
    m2b_add_executable(${target_name} ${ARGN}
        SELECTOR_PROPERTY "M2_SIC_ROBOT_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
    )
endfunction()

function(m2_sic_robot_build_current_dir object_name)
    aux_source_directory(. _srcs)
    m2_sic_robot_add_object_library(${object_name} ${_srcs} ${ARGN})
endfunction()
