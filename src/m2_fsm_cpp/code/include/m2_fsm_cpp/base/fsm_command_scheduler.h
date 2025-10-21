#ifndef M2_FSM_CPP_BASE_FSM_COMMAND_SCHEDULER_H
#define M2_FSM_CPP_BASE_FSM_COMMAND_SCHEDULER_H

#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_fsm_cpp/base/fsm_command_instance_id_allocator.h"
#include "m2_fsm_cpp/base/fsm_subsystem.h"
#include "m2_fsm_cpp/base/fsm_subsystem_instance_id_allocator.h"
#include "trolly/trolly_macro.h"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

namespace m2::fsm {

// forward declaration
class command_scheduler;

class command_scheduler {
private:
    explicit command_scheduler() noexcept = default;

public:
    ~command_scheduler() noexcept = default;

    subsystem_instance_id_t register_subsystem(subsystem_ptr&& subsystem) noexcept;

    template<typename T, typename... Args,
        std::enable_if_t<std::is_base_of<subsystem, T>::value, int> = 0>
    subsystem_instance_id_t register_subsystem(Args&&... args) noexcept
    {
        return register_subsystem(std::make_unique<T>(std::forward<Args>(args)...));
    }

    command_instance_id_t schedule_command(command_ptr&& cmd) noexcept;

    template<typename T, typename... Args,
        std::enable_if_t<std::is_base_of<command, T>::value, int> = 0>
    command_instance_id_t schedule_command(Args&&... args) noexcept
    {
        return schedule_command(std::make_unique<T>(std::forward<Args>(args)...));
    }

    void cancel_command(command_instance_id_t instance_id) noexcept;

    void cancel_all_commands() noexcept;

    TROLLY_NO_DISCARD subsystem* get_subsystem(subsystem_instance_id_t instance_id) noexcept;
    // warning: this returns a pointer to a undefined address if the command has been destroyed after loop
    TROLLY_NO_DISCARD command* get_command(command_instance_id_t instance_id) noexcept;
    TROLLY_NO_DISCARD bool is_scheduled(command_instance_id_t instance_id) noexcept;
    void tick() noexcept;

    TROLLY_NO_DISCARD_INLINE command_instance_id_allocator& command_id_allocator() noexcept
    {
        return cmd_id_allocator_;
    }
    TROLLY_NO_DISCARD_INLINE subsystem_instance_id_allocator& subsystem_id_allocator() noexcept
    {
        return ss_id_allocator_;
    }

    static command_scheduler& instance() noexcept
    {
        if (instance_ == nullptr) {
            instance_ = std::unique_ptr<command_scheduler>(new command_scheduler());
        }
        return *instance_;
    }

private:
    void schedule_pending_commands() noexcept;
    void cancel_pending_commands() noexcept;
    bool is_any_requirements_in_use(command::requirement_store_type& requirements) noexcept;
    bool are_requirements_interruptable(command::requirement_store_type& requirements) noexcept;
    void cancel_requirements(command::requirement_store_type& requirements) noexcept;
    void erase_scheduled_command(command_instance_id_t instance_id) noexcept;

private:
    std::unordered_map<subsystem_instance_id_t, subsystem_ptr> subsystems_;
    std::unordered_map<command_instance_id_t, command_ptr> commands_;

    std::unordered_map<subsystem_instance_id_t, command_instance_id_t> in_use_requirements_;
    std::vector<command_instance_id_t> pending_schedule_cmds_;
    std::vector<command_instance_id_t> pending_cancel_cmds_;
    std::vector<command_instance_id_t> scheduled_cmds_;

    command_instance_id_allocator cmd_id_allocator_;
    subsystem_instance_id_allocator ss_id_allocator_;

    static inline std::unique_ptr<command_scheduler> instance_{nullptr};

    TROLLY_DISALLOW_COPY_AND_MOVE(command_scheduler);
};

#define FSM_GET_SUBSYSTEM(id, subsystem_type) \
    dynamic_cast<subsystem_type*>(fsm::command_scheduler::instance().get_subsystem(id))

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_BASE_FSM_SUBSYSTEM_H
