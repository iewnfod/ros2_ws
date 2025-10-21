# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

m2b_define_selector(M2_VESC_SELECTOR)

function(m2_vesc_set_selectors selectors)
    m2b_set_selectors(M2_VESC_SELECTOR ${selectors})
endfunction()
