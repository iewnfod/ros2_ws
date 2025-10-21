#ifndef M2_FSM_CPP_COMMANDS_FSM_WAIT_UNTIL_COMMAND_H
#define M2_FSM_CPP_COMMANDS_FSM_WAIT_UNTIL_COMMAND_H

#include "m2_fsm_cpp/base/fsm_command.h"

#include <functional>

namespace m2::fsm {

class wait_until_command : public command {
public:
    using condition_func_type = std::function<bool()>;

    explicit wait_until_command(condition_func_type&& func) noexcept
        : command("wait_until_command")
        , func_(std::move(func))
    {}
    ~wait_until_command() noexcept override = default;

    void initialize() noexcept override {}
    void execute() noexcept override {}
    void end(bool /* interrupted */) noexcept override {}
    bool is_finished() noexcept override { return func_(); }

private:
    condition_func_type func_;

    TROLLY_DISALLOW_COPY_AND_MOVE(wait_until_command);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_COMMANDS_FSM_WAIT_UNTIL_COMMAND_H
