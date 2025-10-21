#ifndef M2_SIC_ROBOT_SIC_ROBOT_H
#define M2_SIC_ROBOT_SIC_ROBOT_H

#include "m2_sic_robot/base/sic_robot_define.h"

#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_mode_mux_robot.h"

namespace m2::sic_robot {

class sic_robot : public fsm::ros2_ps_mode_mux_robot {
public:
    explicit sic_robot(
        std::shared_ptr<rclcpp::Node> node, uint32_t hz, const std::string& input_topic) noexcept;
    ~sic_robot() noexcept = default;
};

}  // namespace m2::sic_robot

#endif  // M2_SIC_ROBOT_SIC_ROBOT_H
