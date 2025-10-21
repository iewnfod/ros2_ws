#include "m2_fsm_cpp/commands/composite/fsm_sequential_command_group.h"

#include <cassert>

namespace m2::fsm {

void sequential_command_group::add_command(command_ptr&& cmd) noexcept
{
    assert(!running_ && "Commands cannot be added to a composition while it's running.");
    assert(!cmd->composed() && "Specified command has been composed previously.");

    cmd->set_composed(true);
    add_requirements(cmd->requirements());
    if (cmd->interruption_behaviour() == interruption_behaviour_t::CANCEL_SELF) {
        set_interruption_behaviour(interruption_behaviour_t::CANCEL_SELF);
    }
    cmds_.emplace_back(std::move(cmd));
}

void sequential_command_group::initialize() noexcept
{
    running_ = true;
    curr_cmd_idx_ = 0;
    if (!cmds_.empty()) {
        cmds_[0]->initialize();
    }
}

void sequential_command_group::execute() noexcept
{
    if (TROLLY_UNLIKELY(cmds_.empty())) {
        return;
    }
    auto& cmd = cmds_[curr_cmd_idx_];
    cmd->execute();
    if (cmd->is_finished()) {
        cmd->end(false);
        curr_cmd_idx_++;
        if (curr_cmd_idx_ < cmds_.size()) {
            cmds_[curr_cmd_idx_]->initialize();
        }
    }
}

void sequential_command_group::end(bool interrupted) noexcept
{
    if (interrupted && !cmds_.empty() && running_ && curr_cmd_idx_ < cmds_.size()) {
        cmds_[curr_cmd_idx_]->end(true);
    }
    running_ = false;
    curr_cmd_idx_ = 0;
}

bool sequential_command_group::is_finished() noexcept { return curr_cmd_idx_ == cmds_.size(); }

command* sequential_command_group::get_command(command_instance_id_t instance_id) noexcept
{
    for (auto& cmd : cmds_) {
        if (cmd->instance_id() == instance_id) {
            return cmd.get();
        }
    }
    return nullptr;
}

}  // namespace m2::fsm
