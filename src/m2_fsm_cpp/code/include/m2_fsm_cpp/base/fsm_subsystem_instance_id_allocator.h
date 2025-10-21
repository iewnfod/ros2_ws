#ifndef M2_FSM_CPP_BASE_FSM_SUBSYSTEM_INSTANCE_ID_ALLOCATOR_H
#define M2_FSM_CPP_BASE_FSM_SUBSYSTEM_INSTANCE_ID_ALLOCATOR_H

#include "m2_fsm_cpp/base/fsm_define.h"
#include "m2_fsm_cpp/base/fsm_id_allocator.h"

namespace m2::fsm {

class subsystem_instance_id_allocator : public id_allocator<subsystem_instance_id_t> {
public:
    explicit subsystem_instance_id_allocator() noexcept = default;
    ~subsystem_instance_id_allocator() noexcept override = default;

    TROLLY_DISALLOW_COPY_AND_MOVE(subsystem_instance_id_allocator);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_BASE_FSM_SUBSYSTEM_INSTANCE_ID_ALLOCATOR_H
