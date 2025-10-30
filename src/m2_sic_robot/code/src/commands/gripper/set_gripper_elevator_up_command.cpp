#include "m2_sic_robot/commands/gripper/set_gripper_elevator_up_command.h"
#include "m2_sic_robot/base/sic_robot_define.h"
#include "m2_sic_robot/base/subsystem_container.h"
#include "m2_sic_robot/subsystems/gripper_subsystem.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

namespace m2::sic_robot {

set_gripper_elevator_up_command::set_gripper_elevator_up_command(bool target) noexcept
    : command("set_gripper_elevator_up_command")
	, target_(target)
{
    gripper_subsystem_ =
        FSM_GET_SUBSYSTEM(g_subsystem_container.gripper_subsystem, gripper_subsystem);
    assert(gripper_subsystem_ != nullptr);
    add_requirements({
        g_subsystem_container.gripper_subsystem
        // can add more subsystem requirements here...
    });
}

void set_gripper_elevator_up_command::initialize() noexcept
{
	gripper_subsystem_->set_elevator_up(target_);
    // call gripper_subsystem_'s methods to perform desired actions...
}

void set_gripper_elevator_up_command::execute() noexcept {}

void set_gripper_elevator_up_command::end(bool /* interrupted */) noexcept { TROLLY_INFO("set_gripper_elevator_up_command end."); }

bool set_gripper_elevator_up_command::is_finished() noexcept { return true; }

}  // namespace m2::sic_robot
