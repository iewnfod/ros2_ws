include_guard(GLOBAL)

function(m2_joy_add_interface)
    m2b_add_interface(m2-joy-interface
        INCLUDES ${PROJECT_SOURCE_DIR}/code/include
    )
endfunction()
