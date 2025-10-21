include_guard(GLOBAL)

m2b_define_selector(M2_JOY_SELECTOR)

function(m2_joy_set_selectors selectors)
    m2b_set_selectors(M2_JOY_SELECTOR ${selectors})
endfunction()
