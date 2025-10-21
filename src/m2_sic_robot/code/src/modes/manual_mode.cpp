#include "m2_sic_robot/modes/manual_mode.h"
#include "m2_sic_robot/base/sic_robot_define.h"
#include "m2_sic_robot/commands/example/example_command.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "trolly/log/trolly_logger_macro.h"

namespace m2::sic_robot {

manual_mode::manual_mode() noexcept
    : ps_mode("Manual Mode")
{}

void manual_mode::tick() noexcept {}

void manual_mode::on_enter() noexcept { TROLLY_INFO("[Mode] Entering Manual Mode"); }

void manual_mode::on_exit() noexcept { TROLLY_INFO("[Mode] Exiting Manual Mode"); }

void manual_mode::on_input(const ps_input_data_type& /* data */) noexcept
{
    using namespace fsm::input;
    auto& cs = fsm::command_scheduler::instance();

    // schedule a single command
    if (is_dpad_pressed<ps_dpad_direction_t::LEFT>()) {
        cs.schedule_command<example_command>();
    } else if (is_dpad_pressed<ps_dpad_direction_t::RIGHT>()) {
        cs.schedule_command<example_command>();
    }

    // cancel all existing scheduled commands
    if (is_button_pressed<ps_button_t::OPTIONS>()) {
        cs.cancel_all_commands();
    }
}

}  // namespace m2::sic_robot
