#include "m2_sic_robot/modes/manual_mode.h"
#include <m2_fsm_cpp/robots/ros2/fsm_ros2_ps_input.h>
#include "m2_sic_robot/base/sic_robot_define.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_sic_robot/commands/gripper/set_gripper_elevator_up_command.h"
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

    TROLLY_INFO("[Mode] Controller Input");

    // schedule a single command
    if (is_dpad_pressed<ps_dpad_direction_t::LEFT>()) {
        TROLLY_INFO("Scheduling set_gripper_elevator_up_command to UP");
        cs.schedule_command<set_gripper_elevator_up_command>(true);
    } else if (is_dpad_pressed<ps_dpad_direction_t::RIGHT>()) {
        TROLLY_INFO("Scheduling set_gripper_elevator_up_command to DOWN");
        cs.schedule_command<set_gripper_elevator_up_command>(false);
    }

    // cancel all existing scheduled commands
    if (is_button_pressed<ps_button_t::OPTIONS>()) {
        cs.cancel_all_commands();
    }
}

}  // namespace m2::sic_robot
