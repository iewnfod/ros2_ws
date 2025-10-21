#ifndef M2_SIC_ROBOT_BASE_SUBSYSTEM_CONTAINER_H
#define M2_SIC_ROBOT_BASE_SUBSYSTEM_CONTAINER_H

#include "m2_fsm_cpp/base/fsm_define.h"

namespace m2::sic_robot {

struct subsystem_container_t {
    fsm::subsystem_instance_id_t example_subsystem{fsm::SUBSYSTEM_INSTANCE_ID_INVALID};
    // more susbsystem here...
};

inline subsystem_container_t g_subsystem_container;

}  // namespace m2::sic_robot

#endif  // M2_SIC_ROBOT_BASE_SUBSYSTEM_CONTAINER_H
