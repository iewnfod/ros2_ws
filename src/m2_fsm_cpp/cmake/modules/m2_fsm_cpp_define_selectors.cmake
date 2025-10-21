# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

m2b_define_selector(M2_FSM_CPP_SELECTOR)

function(m2_fsm_cpp_set_selectors selectors)
    m2b_set_selectors(M2_FSM_CPP_SELECTOR ${selectors})
endfunction()
