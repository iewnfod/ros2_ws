#ifndef M2_SIC_ROBOT_COMMANDS_SET_GRIPPER_ELEVATOR_UP_COMMAND_H
#define M2_SIC_ROBOT_COMMANDS_SET_GRIPPER_ELEVATOR_UP_COMMAND_H

#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_sic_robot/subsystems/gripper_subsystem.h"

namespace m2::sic_robot {

class set_gripper_elevator_up_command : public fsm::command {
public:
    explicit set_gripper_elevator_up_command(bool target) noexcept;

    void initialize() noexcept;

    void execute() noexcept;

    void end(bool interrupted) noexcept;

    bool is_finished() noexcept;

private:
    gripper_subsystem* gripper_subsystem_;
	bool target_;
};
}  // namespace m2::sic_robot

#endif  // M2_SIC_ROBOT_COMMANDS_GRIPPER_SET_ELEVATOR_UP_COMMAND_H
