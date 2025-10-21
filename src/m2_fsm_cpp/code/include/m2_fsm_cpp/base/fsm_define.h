#ifndef M2_FSM_CPP_BASE_FSM_DEFINE_H
#define M2_FSM_CPP_BASE_FSM_DEFINE_H

#include <cstdint>
#include <limits>

namespace m2::fsm {

using subsystem_instance_id_t = uint32_t;
using command_instance_id_t = uint32_t;

static constexpr subsystem_instance_id_t SUBSYSTEM_INSTANCE_ID_INVALID =
    std::numeric_limits<subsystem_instance_id_t>::max();
static constexpr command_instance_id_t COMMAND_INSTANCE_ID_INVALID =
    std::numeric_limits<command_instance_id_t>::max();

enum class interruption_behaviour_t {
    CANCEL_SELF = 0,
    CANCEL_INCOMING = 1,
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_BASE_FSM_DEFINE_H
