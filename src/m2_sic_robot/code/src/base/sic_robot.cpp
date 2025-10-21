#include "m2_sic_robot/base/sic_robot.h"

#include "m2_sic_robot/base/subsystem_container.h"
#include "m2_sic_robot/modes/manual_mode.h"
#include "m2_sic_robot/subsystems/example_subsystem.h"

#include "trolly/log/trolly_logger_macro.h"

namespace m2::sic_robot {

sic_robot::sic_robot(
    std::shared_ptr<rclcpp::Node> node, uint32_t hz, const std::string& input_topic) noexcept
    : fsm::ros2_ps_mode_mux_robot(node, hz, input_topic)
{
    auto& cs = fsm::command_scheduler::instance();
    // register subsystems here...
    g_subsystem_container.example_subsystem = cs.register_subsystem<example_subsystem>(node);

    mode_store_type modes;
    // add modes here...
    modes.emplace_back(std::make_unique<manual_mode>());
    add_modes(modes);
}
}  // namespace m2::sic_robot
