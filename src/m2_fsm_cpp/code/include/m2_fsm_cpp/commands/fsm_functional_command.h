#ifndef M2_FSM_CPP_COMMANDS_FSM_FUNCTIONAL_COMMAND_H
#define M2_FSM_CPP_COMMANDS_FSM_FUNCTIONAL_COMMAND_H

#include "m2_fsm_cpp/base/fsm_command.h"

#include <functional>

namespace m2::fsm {

class functional_command : public command {
public:
    using func_type = std::function<void()>;
    using end_func_type = std::function<void(bool)>;
    using is_finished_func_type = std::function<bool()>;

    explicit functional_command(func_type&& initialize_func, func_type&& execute_func,
        end_func_type&& end_func, is_finished_func_type&& is_finished_func) noexcept
        : command("functional_command")
        , initialize_func_(std::move(initialize_func))
        , execute_func_(std::move(execute_func))
        , end_func_(std::move(end_func))
        , is_finished_func_(std::move(is_finished_func))
    {}
    ~functional_command() noexcept override = default;

    void initialize() noexcept override { initialize_func_(); }
    void execute() noexcept override { execute_func_(); }
    void end(bool interrupted) noexcept override { end_func_(interrupted); }
    bool is_finished() noexcept override { return is_finished_func_(); }

private:
    func_type initialize_func_;
    func_type execute_func_;
    end_func_type end_func_;
    is_finished_func_type is_finished_func_;

    TROLLY_DISALLOW_COPY_AND_MOVE(functional_command);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_COMMANDS_FSM_FUNCTIONAL_COMMAND_H
