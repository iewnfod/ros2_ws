#ifndef M2_SIC_ROBOT_SUBSYSTEMS_EXAMPLE_SUBSYSTEM_H
#define M2_SIC_ROBOT_SUBSYSTEMS_EXAMPLE_SUBSYSTEM_H

#include "m2_fsm_cpp/base/fsm_subsystem.h"
#include "m2_sic_robot/base/sic_robot_define.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace m2::sic_robot {

class example_subsystem : public fsm::subsystem {
public:
    explicit example_subsystem(std::shared_ptr<rclcpp::Node> node) noexcept;
    ~example_subsystem() noexcept = default;

    void tick() noexcept override;

private:
    void example_sub_cb(const std_msgs::msg::Float32& msg) noexcept;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr example_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr example_sub_;

    TROLLY_DISALLOW_COPY_AND_MOVE(example_subsystem);
};

}  // namespace m2::sic_robot

#endif  // M2_SIC_ROBOT_SUBSYSTEMS_EXAMPLE_SUBSYSTEM_H
