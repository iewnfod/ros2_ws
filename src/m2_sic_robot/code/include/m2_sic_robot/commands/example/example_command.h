#ifndef M2_SIC_ROBOT_COMMANDS_EXAMPLE_COMMAND_H
#define M2_SIC_ROBOT_COMMANDS_EXAMPLE_COMMAND_H

#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_sic_robot/subsystems/example_subsystem.h"

namespace m2::sic_robot {

class example_command : public fsm::command {
public:
    explicit example_command() noexcept;

    void initialize() noexcept;

    void execute() noexcept;

    void end(bool interrupted) noexcept;

    bool is_finished() noexcept;

private:
    example_subsystem* example_subsystem_;
};
}  // namespace m2::sic_robot

#endif  // M2_SIC_ROBOT_COMMANDS_EXAMPLE_COMMAND_H
