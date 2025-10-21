include_guard(GLOBAL)

find_package(trolly REQUIRED)

function(m2_joy_add_library target_name)
    m2b_add_library(${target_name} SHARED ${ARGN}
        SELECTOR_PROPERTY "M2_JOY_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
    )
endfunction()

function(m2_joy_add_object_library object_name)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
    m2b_add_object_library(${object_name} ${ARG_UNPARSED_ARGUMENTS}
        DEPENDS
            m2-joy-interface
            trolly::trolly-interface
            ${ARG_DEPENDS}
    )
endfunction()

function(m2_joy_add_executable target_name)
    m2b_add_executable(${target_name} ${ARGN}
        SELECTOR_PROPERTY "M2_JOY_SELECTOR"
        WHOLE_DEPENDS
            trolly::trolly
    )
endfunction()

function(m2_joy_build_current_dir object_name)
    aux_source_directory(. _srcs)
    m2_joy_add_object_library(${object_name} ${_srcs} ${ARGN})
endfunction()
