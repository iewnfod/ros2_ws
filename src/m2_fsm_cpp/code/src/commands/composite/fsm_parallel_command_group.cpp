#include "m2_fsm_cpp/commands/composite/fsm_parallel_command_group.h"

#include <cassert>

namespace m2::fsm {

bool parallel_command_group::is_running() noexcept
{
    for (const auto& kvp : running_cmds_) {
        if (kvp.second) {
            return true;
        }
    }
    return false;
}

void parallel_command_group::add_command(command_ptr&& cmd) noexcept
{
    assert(!is_running() && "Commands cannot be added to a composition while it's running.");
    assert(!cmd->composed() && "Specified command has been composed previously.");

    cmd->set_composed(true);
    add_requirements(cmd->requirements());
    if (cmd->interruption_behaviour() == interruption_behaviour_t::CANCEL_SELF) {
        set_interruption_behaviour(interruption_behaviour_t::CANCEL_SELF);
    }
    running_cmds_[std::move(cmd)] = false;
}

void parallel_command_group::initialize() noexcept
{
    for (auto& kvp : running_cmds_) {
        kvp.first->initialize();
        kvp.second = true;
    }
}

void parallel_command_group::execute() noexcept
{
    for (auto& kvp : running_cmds_) {
        const auto& cmd = kvp.first;
        bool& running = kvp.second;
        if (!running) {
            continue;
        }
        cmd->execute();
        if (cmd->is_finished()) {
            cmd->end(false);
            running = false;
        }
    }
}

void parallel_command_group::end(bool interrupted) noexcept
{
    if (interrupted) {
        for (auto& kvp : running_cmds_) {
            if (kvp.second) {
                kvp.first->end(true);
            }
        }
    }
}

bool parallel_command_group::is_finished() noexcept { return !is_running(); }

command* parallel_command_group::get_command(command_instance_id_t instance_id) noexcept
{
    for (auto& kvp : running_cmds_) {
        if (kvp.first->instance_id() == instance_id) {
            return kvp.first.get();
        }
    }
    return nullptr;
}

}  // namespace m2::fsm
