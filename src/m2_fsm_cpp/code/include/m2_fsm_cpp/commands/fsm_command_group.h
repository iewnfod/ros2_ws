#ifndef M2_FSM_CPP_COMMANDS_FSM_COMMAND_GROUP_H
#define M2_FSM_CPP_COMMANDS_FSM_COMMAND_GROUP_H

#include "m2_fsm_cpp/base/fsm_command.h"

namespace m2::fsm {

class command_group : public command {
public:
    explicit command_group(const std::string& name) noexcept
        : command(name)
    {}
    ~command_group() noexcept override = default;

    // base case
    template<typename... Args>
    void add_commands() noexcept
    {}

    template<typename... Args>
    void add_commands(command_ptr&& first, Args&&... args) noexcept
    {
        add_command(std::move(first));
        add_commands(std::forward<Args>(args)...);
    }

protected:
    virtual void add_command(command_ptr&& cmd) noexcept = 0;

    TROLLY_DISALLOW_COPY_AND_MOVE(command_group);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_COMMANDS_FSM_COMMAND_GROUP_H
