#include "m2_fsm_cpp/base/fsm_command.h"

#include <algorithm>

namespace m2::fsm {

bool command::has_requirement(const subsystem& subsystem) noexcept
{
    return has_requirement(subsystem.instance_id());
}

bool command::has_requirement(subsystem_instance_id_t instance_id) noexcept
{
    return std::find(requirements_.begin(), requirements_.end(), instance_id)
        != requirements_.end();
}

}  // namespace m2::fsm
