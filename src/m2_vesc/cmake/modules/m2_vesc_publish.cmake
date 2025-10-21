# Copyright (c) 2024 HKU Robocon Team
# Author: Anthony Law (anthlaw@connect.hku.hk)

include_guard(GLOBAL)

m2_vesc_add_interface()
m2_vesc_add_library(m2-vesc
    SELECTORS
        standalone
)

m2b_install_targets(m2-vesc m2-vesc-interface)
