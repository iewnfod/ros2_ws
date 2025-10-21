#ifndef M2_FSM_CPP_BASE_FSM_ROBOT_H
#define M2_FSM_CPP_BASE_FSM_ROBOT_H

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_fsm_cpp/base/fsm_define.h"
#include "trolly/trolly_macro.h"

namespace m2::fsm {

class robot {
public:
    explicit robot() noexcept
        : cmd_scheduler_(&command_scheduler::instance())
    {}
    virtual ~robot() noexcept = default;

    virtual void initialize() noexcept = 0;
    virtual void tick() noexcept { cmd_scheduler_->tick(); }

private:
    command_scheduler* cmd_scheduler_;

    TROLLY_DISALLOW_COPY_AND_MOVE(robot);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_BASE_FSM_ROBOT_H
