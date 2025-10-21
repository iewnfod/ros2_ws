#ifndef M2_FSM_CPP_BASE_FSM_COMMAND_H
#define M2_FSM_CPP_BASE_FSM_COMMAND_H

#include "m2_fsm_cpp/base/fsm_define.h"
#include "m2_fsm_cpp/base/fsm_subsystem.h"
#include "trolly/trolly_macro.h"

#include <mutex>
#include <set>
#include <string>

namespace m2::fsm {

class command;
using command_ptr = std::unique_ptr<command>;

class command {
public:
    using requirement_store_type = std::set<subsystem_instance_id_t>;

    explicit command(const std::string& name) noexcept
        : instance_id_(COMMAND_INSTANCE_ID_INVALID)
        , name_(name){};
    virtual ~command() noexcept = default;

    TROLLY_ALWAYS_INLINE void set_instance_id(subsystem_instance_id_t instance_id)
    {
        instance_id_ = instance_id;
    }
    TROLLY_NO_DISCARD_INLINE command_instance_id_t instance_id() const noexcept
    {
        return instance_id_;
    }
    TROLLY_NO_DISCARD_INLINE std::string name() const noexcept { return name_; }
    TROLLY_ALWAYS_INLINE void set_name(const std::string& name) noexcept { name_ = name; }
    TROLLY_NO_DISCARD_INLINE bool composed() const noexcept { return composed_; }

    virtual interruption_behaviour_t interruption_behaviour() noexcept
    {
        return interruption_behaviour_t::CANCEL_SELF;
    }

    template<typename T, typename... Args,
        std::enable_if_t<std::is_base_of<subsystem, T>::value, int> = 0>
    void add_requirements(T& first, Args&&... args) noexcept
    {
        requirements_.insert(first.instance_id());
        if constexpr (sizeof...(Args) > 0) {
            add_requirements(std::forward<Args>(args)...);
        }
    }

    void add_requirements(const requirement_store_type& requirements) noexcept
    {
        requirements_.insert(requirements.begin(), requirements.end());
    }

    TROLLY_NO_DISCARD_INLINE requirement_store_type& requirements() noexcept
    {
        return requirements_;
    }
    TROLLY_NO_DISCARD bool has_requirement(const subsystem& subsystem) noexcept;
    TROLLY_NO_DISCARD bool has_requirement(subsystem_instance_id_t instance_id) noexcept;
    TROLLY_ALWAYS_INLINE void set_composed(bool composed) noexcept { composed_ = composed; }

    virtual void initialize() noexcept = 0;
    virtual void execute() noexcept = 0;
    virtual void end(bool interrupted) noexcept = 0;
    TROLLY_NO_DISCARD virtual bool is_finished() noexcept = 0;

private:
    command_instance_id_t instance_id_;
    std::string name_;
    bool composed_{false};
    // uses subsystem name for setting requirements
    requirement_store_type requirements_;

    TROLLY_DISALLOW_COPY_AND_MOVE(command);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_BASE_FSM_COMMAND_H
