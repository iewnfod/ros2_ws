#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

#include "trolly/log/trolly_logger_macro.h"

namespace m2::fsm {

subsystem_instance_id_t command_scheduler::register_subsystem(subsystem_ptr&& subsystem) noexcept
{
    auto instance_id = subsystem->instance_id();
    if (instance_id == SUBSYSTEM_INSTANCE_ID_INVALID) {
        instance_id = ss_id_allocator_.allocate_id();
        subsystem->set_instance_id(instance_id);
    }
    subsystems_[instance_id] = std::move(subsystem);
    return instance_id;
}

command_instance_id_t command_scheduler::schedule_command(command_ptr&& cmd) noexcept
{
    if (cmd->composed()) {
        return COMMAND_INSTANCE_ID_INVALID;
    }

    auto instance_id = cmd->instance_id();
    if (instance_id == COMMAND_INSTANCE_ID_INVALID) {
        instance_id = cmd_id_allocator_.allocate_id();
        cmd->set_instance_id(instance_id);
    }
    pending_schedule_cmds_.emplace_back(instance_id);
    commands_[instance_id] = std::move(cmd);
    return instance_id;
}

void command_scheduler::cancel_command(command_instance_id_t instance_id) noexcept
{
    if (commands_.find(instance_id) == commands_.end()) {
        TROLLY_WARN(
            "Command with instance id %d was asked to cancel, but it does not exist, ignoring",
            instance_id);
        return;
    }
    pending_cancel_cmds_.emplace_back(instance_id);
}

void command_scheduler::cancel_all_commands() noexcept
{
    for (auto& kvp : commands_) {
        pending_cancel_cmds_.emplace_back(kvp.first);
    }
}

bool command_scheduler::is_scheduled(command_instance_id_t instance_id) noexcept
{
    return std::find(scheduled_cmds_.begin(), scheduled_cmds_.end(), instance_id)
        != scheduled_cmds_.end();
}

subsystem* command_scheduler::get_subsystem(subsystem_instance_id_t instance_id) noexcept
{
    if (subsystems_.find(instance_id) == subsystems_.end()) {
        return nullptr;
    }
    return subsystems_[instance_id].get();
}

command* command_scheduler::get_command(command_instance_id_t instance_id) noexcept
{
    if (commands_.find(instance_id) == commands_.end()) {
        return nullptr;
    }
    return commands_[instance_id].get();
}

bool command_scheduler::is_any_requirements_in_use(
    command::requirement_store_type& requirements) noexcept
{
    for (const auto& ss_inst_id : requirements) {
        if (in_use_requirements_.find(ss_inst_id) != in_use_requirements_.end()) {
            return true;
        }
    }
    return false;
}

bool command_scheduler::are_requirements_interruptable(
    command::requirement_store_type& requirements) noexcept
{
    for (const auto& ss_inst_id : requirements) {
        auto it = in_use_requirements_.find(ss_inst_id);
        if (it == in_use_requirements_.end()) {
            continue;
        }

        auto* curr_cmd = get_command(it->second);
        if (curr_cmd == nullptr) {
            continue;
        }

        if (curr_cmd->interruption_behaviour() == interruption_behaviour_t::CANCEL_INCOMING) {
            return false;
        }
    }
    return true;
}

void command_scheduler::cancel_requirements(command::requirement_store_type& requirements) noexcept
{
    // this function expects:
    // - the command exists
    // - the command is interruptable
    for (const auto& ss_inst_id : requirements) {
        auto it = in_use_requirements_.find(ss_inst_id);
        if (it == in_use_requirements_.end()) {
            continue;
        }
        cancel_command(it->second);
        in_use_requirements_.erase(it);
    }
}

template<typename VectorType, typename ValueType>
static void erase_existing_from_vector(VectorType& vector, ValueType& value) noexcept
{
    auto it = std::find(vector.begin(), vector.end(), value);
    if (TROLLY_LIKELY(it != vector.end())) {
        vector.erase(it);
    }
}

void command_scheduler::schedule_pending_commands() noexcept
{
    std::vector<command_instance_id_t> to_remove;
    for (auto& cmd_inst_id : pending_schedule_cmds_) {
        auto* cmd = get_command(cmd_inst_id);
        if (TROLLY_UNLIKELY(cmd == nullptr)) {
            continue;
        }

        if (is_scheduled(cmd_inst_id)) {
            continue;
        }

        // check for requirements
        auto requirements = cmd->requirements();
        if (is_any_requirements_in_use(requirements)) {
            if (!are_requirements_interruptable(requirements)) {
                // skip if not interruptable
                continue;
            }
            cancel_requirements(requirements);
        }

        // add requirements to in use
        for (const auto& ss_inst_id : requirements) {
            in_use_requirements_[ss_inst_id] = cmd_inst_id;
        }

        // schedule and init command
        scheduled_cmds_.emplace_back(cmd_inst_id);
        to_remove.emplace_back(cmd_inst_id);
        TROLLY_DEBUG("command %s.initialize() start", cmd->name().c_str());
        cmd->initialize();
        TROLLY_DEBUG("command %s.initialize() end", cmd->name().c_str());
    }
    for (auto& cmd_inst_id : to_remove) {
        erase_existing_from_vector(pending_schedule_cmds_, cmd_inst_id);
    }
}

void command_scheduler::erase_scheduled_command(command_instance_id_t instance_id) noexcept
{
    // TODO(Anthony Law): deallocate id
    // remove from scheduled cmds list and release the resources of the command
    erase_existing_from_vector(scheduled_cmds_, instance_id);
    commands_.erase(instance_id);
}

void command_scheduler::cancel_pending_commands() noexcept
{
    for (auto& cmd_inst_id : pending_cancel_cmds_) {
        if (!is_scheduled(cmd_inst_id)) {
            continue;
        }

        auto* cmd = get_command(cmd_inst_id);
        if (TROLLY_UNLIKELY(cmd == nullptr)) {
            continue;
        }

        // release in use requirements
        for (const auto& req : cmd->requirements()) {
            if (TROLLY_LIKELY(in_use_requirements_.find(req) != in_use_requirements_.end())) {
                in_use_requirements_.erase(req);
            }
        }
        cmd->end(true);
        erase_scheduled_command(cmd_inst_id);
    }
    pending_cancel_cmds_.clear();
}

void command_scheduler::tick() noexcept
{
    // perform subsystem tick
    for (auto& kvp : subsystems_) {
        auto& ss = kvp.second;
        TROLLY_DEBUG("subsystem %s->tick() start", ss->name().c_str());
        ss->tick();
        TROLLY_DEBUG("subsystem %s->tick() end", ss->name().c_str());
    }

    // main loop
    std::vector<command_instance_id_t> to_remove;
    for (auto& cmd_inst_id : scheduled_cmds_) {
        auto* cmd = get_command(cmd_inst_id);
        if (TROLLY_UNLIKELY(cmd == nullptr)) {
            to_remove.emplace_back(cmd_inst_id);
            continue;
        }

        TROLLY_DEBUG("command %s->execute() start", cmd->name().c_str());
        cmd->execute();
        TROLLY_DEBUG("command %s->execute() end", cmd->name().c_str());

        TROLLY_DEBUG("command %s->is_finished() start", cmd->name().c_str());
        if (cmd->is_finished()) {
            TROLLY_DEBUG("command %s->is_finished() end", cmd->name().c_str());
            TROLLY_DEBUG("command %s->end() start", cmd->name().c_str());
            cmd->end(false);
            TROLLY_DEBUG("command %s->end() end", cmd->name().c_str());
            to_remove.emplace_back(cmd_inst_id);
            // release in use requirements
            for (const auto& req : cmd->requirements()) {
                if (TROLLY_LIKELY(in_use_requirements_.find(req) != in_use_requirements_.end())) {
                    in_use_requirements_.erase(req);
                }
            }
        }
    }

    // remove completed commands
    for (auto& cmd_inst_id : to_remove) {
        erase_scheduled_command(cmd_inst_id);
    }

    // schedule/cancel commands queued during loop
    schedule_pending_commands();
    cancel_pending_commands();
}

}  // namespace m2::fsm
