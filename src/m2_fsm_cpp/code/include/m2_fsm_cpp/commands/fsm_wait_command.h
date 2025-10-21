#ifndef M2_FSM_CPP_COMMANDS_FSM_WAIT_COMMAND_H
#define M2_FSM_CPP_COMMANDS_FSM_WAIT_COMMAND_H

#include "m2_fsm_cpp/base/fsm_command.h"

#include <chrono>

namespace m2::fsm {

// TODO(Anthony Law): shouldn't use realtime, should change to use tick time
class wait_command : public command {
public:
    explicit wait_command(uint32_t ms) noexcept
        : command("wait_command")
        , ms_(ms)
    {}
    ~wait_command() noexcept override = default;

    void initialize() noexcept override { start_time_ = std::chrono::steady_clock::now(); }
    void execute() noexcept override {}
    void end(bool /* interrupted */) noexcept override {}
    bool is_finished() noexcept override
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start_time_)
                   .count()
            >= ms_;
    }

private:
    uint32_t ms_;
    std::chrono::steady_clock::time_point start_time_;

    TROLLY_DISALLOW_COPY_AND_MOVE(wait_command);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_COMMANDS_FSM_WAIT_COMMAND_H
