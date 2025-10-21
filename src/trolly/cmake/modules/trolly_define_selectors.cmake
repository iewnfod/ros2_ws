# Copyright (c) 2025 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

m2b_define_selector(TROLLY_SELECTOR)

function(trolly_set_selectors selectors)
    m2b_set_selectors(TROLLY_SELECTOR ${selectors})
endfunction()
