include_guard(GLOBAL)

m2b_define_selector(M2_SIC_ROBOT_SELECTOR)

function(m2_sic_robot_set_selectors selectors)
    m2b_set_selectors(M2_SIC_ROBOT_SELECTOR ${selectors})
endfunction()
