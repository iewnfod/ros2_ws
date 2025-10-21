#include "m2_sic_robot/commands/example/example_command.h"
#include "m2_sic_robot/base/sic_robot_define.h"
#include "m2_sic_robot/base/subsystem_container.h"
#include "m2_sic_robot/subsystems/example_subsystem.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

namespace m2::sic_robot {

example_command::example_command() noexcept
    : command("example_command")
{
    example_subsystem_ =
        FSM_GET_SUBSYSTEM(g_subsystem_container.example_subsystem, example_subsystem);
    assert(example_subsystem_ != nullptr);
    add_requirements({
        g_subsystem_container.example_subsystem,
        // can add more subsystem requirements here...
    });
}

void example_command::initialize() noexcept
{
    // call example_subsystem_'s methods to perform desired actions...
}

void example_command::execute() noexcept {}

void example_command::end(bool /* interrupted */) noexcept { TROLLY_INFO("example_command end."); }

bool example_command::is_finished() noexcept { return true; }

}  // namespace m2::sic_robot
