include_guard(GLOBAL)

m2_sic_robot_add_interface()

m2_sic_robot_add_library(m2-sic-robot
    SELECTORS
        m2-sic-robot
)

m2b_install_targets(m2-sic-robot m2-sic-robot-interface)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
