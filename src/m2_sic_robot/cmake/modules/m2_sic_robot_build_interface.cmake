include_guard(GLOBAL)

function(m2_sic_robot_add_interface)
    m2b_add_interface(m2-sic-robot-interface
        INCLUDES
            ${PROJECT_SOURCE_DIR}/code/include
    )
endfunction()
