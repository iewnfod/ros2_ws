#ifndef M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_ROBOT_H
#define M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_ROBOT_H

#include "m2_fsm_cpp/base/fsm_robot.h"

#include <rclcpp/rclcpp.hpp>

namespace m2::fsm {

inline constexpr uint32_t SECONDS_TO_MILLISECONDS = 1000U;

class ros2_robot : public robot {
public:
    explicit ros2_robot(std::shared_ptr<rclcpp::Node> node, uint32_t hz) noexcept
        : node_(std::move(node))
        , hz_(hz)
    {}
    ~ros2_robot() noexcept override = default;

    virtual void start() noexcept;

protected:
    rclcpp::Node* node_handle() noexcept { return node_.get(); }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint32_t hz_;

    TROLLY_DISALLOW_COPY_AND_MOVE(ros2_robot);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_ROBOT_H
