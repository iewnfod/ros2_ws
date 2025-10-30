#include "m2_sic_robot/subsystems/gripper_subsystem.h"
#include "m2_sic_robot/base/sic_robot_define.h"

#include "trolly/log/trolly_logger_macro.h"
#include "trolly/math/trolly_math_util.h"

#include "std_msgs/msg/bool.hpp"

namespace m2::sic_robot {

gripper_subsystem::gripper_subsystem(std::shared_ptr<rclcpp::Node> node) noexcept
    : subsystem("gripper_subsystem")
    , node_(node)
{
    node_->declare_parameter("gripper_set_topic", "gripper/setpoint");
    node_->declare_parameter("gripper_fb_topic", "gripper/feedback");

    auto gripper_set_topic = node_->get_parameter("gripper_set_topic").as_string();
    auto gripper_fb_topic = node_->get_parameter("gripper_fb_topic").as_string();

    // publishers here...
    gripper_pub_ = node_->create_publisher<std_msgs::msg::Bool>(gripper_set_topic, 1);

    // subscribers here...
    gripper_sub_ = node->create_subscription<std_msgs::msg::Bool>(gripper_fb_topic, 1,
        [this](const std_msgs::msg::Bool& msg) { this->gripper_sub_cb(msg); });

    gpiod::chip chip();
    elevator_ = gpiod::find_line("GPIO25");
    
    
}

void gripper_subsystem::gripper_sub_cb(const std_msgs::msg::Bool& msg) noexcept
{
    (void)msg;  // process msg here
}

void gripper_subsystem::tick() noexcept {}

void gripper_subsystem::set_elevator_up(bool up) noexcept
{
	try {
		elevator_.set_value(up ? 1 : 0);
	} catch (const std::exception& e) {
		TROLLY_ERROR("Failed to set elevator line value: {}", e.what());
	}
}

}  // namespace m2::sic_robot
