#include "m2_fsm_cpp/robots/ros2/fsm_ros2_robot.h"

#include <iostream>

namespace m2::fsm {

void ros2_robot::start() noexcept
{
    initialize();
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(SECONDS_TO_MILLISECONDS / hz_), [this]() { tick(); });
    rclcpp::spin(node_);
    rclcpp::shutdown();
}

}  // namespace m2::fsm
