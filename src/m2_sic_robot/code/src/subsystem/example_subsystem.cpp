#include "m2_sic_robot/subsystems/example_subsystem.h"
#include "m2_sic_robot/base/sic_robot_define.h"

#include "trolly/log/trolly_logger_macro.h"
#include "trolly/math/trolly_math_util.h"

#include "std_msgs/msg/float32.hpp"

namespace m2::sic_robot {

example_subsystem::example_subsystem(std::shared_ptr<rclcpp::Node> node) noexcept
    : subsystem("example_subsystem")
    , node_(node)
{
    node_->declare_parameter("example_set_topic", "example/setpoint");
    node_->declare_parameter("example_fb_topic", "example/feedback");

    auto example_set_topic = node_->get_parameter("example_set_topic").as_string();
    auto example_fb_topic = node_->get_parameter("example_fb_topic").as_string();

    // publishers here...
    example_pub_ = node_->create_publisher<std_msgs::msg::Float32>(example_set_topic, 1);

    // subscribers here...
    example_sub_ = node->create_subscription<std_msgs::msg::Float32>(example_fb_topic, 1,
        [this](const std_msgs::msg::Float32& msg) { this->example_sub_cb(msg); });
}

void example_subsystem::example_sub_cb(const std_msgs::msg::Float32& msg) noexcept
{
    (void)msg;  // process msg here
}

void example_subsystem::tick() noexcept {}

}  // namespace m2::sic_robot
