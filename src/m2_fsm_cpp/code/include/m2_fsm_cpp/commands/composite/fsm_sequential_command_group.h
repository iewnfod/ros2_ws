#ifndef M2_FSM_CPP_COMMANDS_COMPOSITE_FSM_SEQUENTIAL_COMMAND_GROUP_H
#define M2_FSM_CPP_COMMANDS_COMPOSITE_FSM_SEQUENTIAL_COMMAND_GROUP_H

#include "m2_fsm_cpp/commands/fsm_command_group.h"

#include <vector>

namespace m2::fsm {

class sequential_command_group : public command_group {
public:
    explicit sequential_command_group(
        interruption_behaviour_t interruption_behaviour = interruption_behaviour_t::CANCEL_INCOMING)
        : command_group("sequential_command_group")
        , interruption_behaviour_(interruption_behaviour)
    {}

    template<typename... Args>
    explicit sequential_command_group(Args&&... args)
        : command_group("sequential_command_group")
        , interruption_behaviour_(interruption_behaviour_t::CANCEL_INCOMING)
    {
        move_commands(std::forward<Args>(args)...);
    }

    ~sequential_command_group() noexcept override = default;

    void initialize() noexcept override;
    void execute() noexcept override;
    void end(bool interrupted) noexcept override;
    bool is_finished() noexcept override;

    interruption_behaviour_t interruption_behaviour() noexcept override
    {
        return interruption_behaviour_;
    }

    TROLLY_ALWAYS_INLINE void set_interruption_behaviour(
        interruption_behaviour_t interruption_behaviour) noexcept
    {
        interruption_behaviour_ = interruption_behaviour;
    }

    // This returns the commands owned by this group
    TROLLY_NO_DISCARD command* get_command(command_instance_id_t instance_id) noexcept;

protected:
    void add_command(command_ptr&& cmd) noexcept override;

private:
    // base case
    template<typename... Args>
    void move_commands()
    {}

    template<typename... Args>
    void move_commands(command_ptr&& first, Args&&... args)
    {
        add_command(std::move(first));
        move_commands(std::forward<Args>(args)...);
    }

private:
    std::vector<command_ptr> cmds_;
    size_t curr_cmd_idx_{0};
    bool running_{false};
    interruption_behaviour_t interruption_behaviour_;

    TROLLY_DISALLOW_COPY_AND_MOVE(sequential_command_group);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_COMMANDS_COMPOSITE_FSM_SEQUENTIAL_COMMAND_GROUP_H
