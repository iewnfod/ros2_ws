#ifndef M2_FSM_CPP_BASE_FSM_SUBSYSTEM_H
#define M2_FSM_CPP_BASE_FSM_SUBSYSTEM_H

#include "m2_fsm_cpp/base/fsm_define.h"
#include "trolly/trolly_macro.h"

#include <memory>
#include <mutex>
#include <string>

namespace m2::fsm {

class subsystem;
using subsystem_ptr = std::unique_ptr<subsystem>;

class subsystem {
public:
    explicit subsystem(const std::string& name) noexcept
        : instance_id_(SUBSYSTEM_INSTANCE_ID_INVALID)
        , name_(name){};
    virtual ~subsystem() noexcept = default;

    TROLLY_ALWAYS_INLINE void set_instance_id(subsystem_instance_id_t instance_id)
    {
        instance_id_ = instance_id;
    }
    TROLLY_NO_DISCARD_INLINE subsystem_instance_id_t instance_id() const noexcept
    {
        return instance_id_;
    }
    TROLLY_NO_DISCARD_INLINE std::string name() const noexcept { return name_; }
    TROLLY_ALWAYS_INLINE void set_name(const std::string& name) noexcept { name_ = name; }

    virtual void tick() noexcept = 0;

private:
    subsystem_instance_id_t instance_id_;
    std::string name_;

    TROLLY_DISALLOW_COPY_AND_MOVE(subsystem);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_BASE_FSM_SUBSYSTEM_H
