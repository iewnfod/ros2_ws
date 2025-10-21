#ifndef M2_FSM_CPP_COMMANDS_FSM_SCHEDULE_COMMAND_H
#define M2_FSM_CPP_COMMANDS_FSM_SCHEDULE_COMMAND_H

#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

#include <vector>

namespace m2::fsm {

class schedule_command : public command {
public:
    template<typename... Args>
    explicit schedule_command(Args&&... args) noexcept
        : command("schedule_command")
    {
        move_commands(std::forward<Args>(args)...);
    }

public:
    ~schedule_command() noexcept override = default;

    void initialize() noexcept override
    {
        auto& cs = command_scheduler::instance();
        for (auto& cmd : cmds_) {
            cs.schedule_command(std::move(cmd));
        }
    }
    void execute() noexcept override {}
    void end(bool /* interrupted */) noexcept override {}
    bool is_finished() noexcept override { return true; }

private:
    // base case
    template<typename... Args>
    void move_commands()
    {}

    template<typename... Args>
    void move_commands(command_ptr&& first, Args&&... args)
    {
        cmds_.emplace_back(std::move(first));
        move_commands(std::forward<Args>(args)...);
    }

private:
    std::vector<command_ptr> cmds_;

    TROLLY_DISALLOW_COPY_AND_MOVE(schedule_command);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_COMMANDS_FSM_SCHEDULE_COMMAND_H
