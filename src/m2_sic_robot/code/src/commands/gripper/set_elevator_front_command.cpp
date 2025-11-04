#include "m2_sic_robot/commands/gripper/set_elevator_front_command.h"
#include "m2_sic_robot/base/sic_robot_define.h"
#include "m2_sic_robot/base/subsystem_container.h"
#include "m2_sic_robot/subsystems/gripper_subsystem.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

namespace m2::sic_robot {

set_elevator_front_command::set_elevator_front_command(bool target) noexcept
    : command("set_elevator_front_command")
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

void set_elevator_front_command::initialize() noexcept
{
	gripper_subsystem_->set_elevator_front(target_);
    TROLLY_INFO("set_elevator_front_command set to %d.", target_);
    // call gripper_subsystem_'s methods to perform desired actions...
}

void set_elevator_front_command::execute() noexcept {}

void set_elevator_front_command::end(bool /* interrupted */) noexcept { TROLLY_INFO("set_elevator_front_command end."); }

bool set_elevator_front_command::is_finished() noexcept { return true; }

}  // namespace m2::sic_robot
