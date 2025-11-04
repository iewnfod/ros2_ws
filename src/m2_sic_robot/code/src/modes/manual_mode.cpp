#include "m2_sic_robot/modes/manual_mode.h"
#include <m2_fsm_cpp/robots/ros2/fsm_ros2_ps_input.h>
#include "m2_sic_robot/base/sic_robot_define.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_sic_robot/commands/gripper/set_elevator_back_command.h"
#include "m2_sic_robot/commands/gripper/set_elevator_front_command.h"
#include "m2_sic_robot/commands/gripper/set_gripper_command.h"
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

    // TROLLY_INFO("[Mode] Controller Input: Square: %d, Cross: %d, Circle: %d, Triangle: %d, Options: %d",
    //     data.square, data.cross, data.circle, data.triangle, data.options);

    // schedule a single command
    // Square = X, Cross = A, Circle = B, Triangle = Y
    if (is_button_pressed<ps_button_t::CIRCLE>()) {  // X
        TROLLY_INFO("Button Circle Pressed");
        if (!elevator_front_state_)
        {
            elevator_front_state_ = true;
            cs.schedule_command<set_elevator_front_command>(true);
        }
    } else if (is_button_pressed<ps_button_t::SQUARE>()) {  // B
        TROLLY_INFO("Button Square Pressed");
        if (elevator_front_state_)
        {
            elevator_front_state_ = false;
            cs.schedule_command<set_elevator_front_command>(false);
        }
    }

    if (is_dpad_pressed<ps_dpad_direction_t::UP>()) {
        TROLLY_INFO("DPad UP Pressed");
        if (!elevator_back_state_)
        {
            elevator_back_state_ = true;
            cs.schedule_command<set_elevator_back_command>(true);
        }
    } else if (is_dpad_pressed<ps_dpad_direction_t::DOWN>()) {
        TROLLY_INFO("DPad DOWN Pressed");
        if (elevator_back_state_)
        {
            elevator_back_state_ = false;
            cs.schedule_command<set_elevator_back_command>(false);
        }
    }

    if (is_button_pressed<ps_button_t::L1>()) {
        TROLLY_INFO("Button L1 Pressed");
        if (!gripper_state_)
        {
            gripper_state_ = true;
            cs.schedule_command<set_gripper_command>(true);
        }
    } else if (is_button_pressed<ps_button_t::R1>()) {
        TROLLY_INFO("Button R1 Pressed");
        if (gripper_state_)
        {
            gripper_state_ = false;
            cs.schedule_command<set_gripper_command>(false);
        }
    }

    // cancel all existing scheduled commands
    if (is_button_pressed<ps_button_t::OPTIONS>()) {
        cs.cancel_all_commands();
    }
}

}  // namespace m2::sic_robot
