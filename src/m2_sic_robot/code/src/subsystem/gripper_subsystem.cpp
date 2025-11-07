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
    auto chip = gpiod::chip("gpiochip4");

    elevator_front_ = chip.get_line(18);
    elevator_front_.request(
        {"elevator_front_",
            gpiod::line_request::DIRECTION_OUTPUT,
        0},
        false
    );
    
    elevator_back_ = chip.get_line(23);
    elevator_back_.request(
        {"elevator_back_",
            gpiod::line_request::DIRECTION_OUTPUT,
        0},
        false
    );

    gripper_ = chip.get_line(25);
    gripper_.request(
        {"gripper_",
            gpiod::line_request::DIRECTION_OUTPUT,
        0},
        false
    );
}

void gripper_subsystem::release_lines() noexcept{
    elevator_front_.release();
    elevator_back_.release();
    gripper_.release();
}

void gripper_subsystem::gripper_sub_cb(const std_msgs::msg::Bool& msg) noexcept
{
    (void)msg;  // process msg here
}

void gripper_subsystem::tick() noexcept {}

void gripper_subsystem::set_elevator_front(bool up) noexcept
{
	try {
		elevator_front_.set_value(up ? 1 : 0);
        TROLLY_INFO("Set elevator front value: %d", up);
	} catch (const std::exception& e) {
		TROLLY_ERROR("Failed to set elevator line value: %s", e.what());
	}
}

void gripper_subsystem::set_elevator_back(bool up) noexcept
{
	try {
		elevator_back_.set_value(up ? 1 : 0);
        TROLLY_INFO("Set elevator back value: %d", up);
	} catch (const std::exception& e) {
		TROLLY_ERROR("Failed to set elevator line value: %s", e.what());
	}
}

void gripper_subsystem::set_gripper(bool up) noexcept
{
	try {
		gripper_.set_value(up ? 1 : 0);
        TROLLY_INFO("Set gripper value: %d", up);
	} catch (const std::exception& e) {
		TROLLY_ERROR("Failed to set elevator line value: %s", e.what());
	}
}

}  // namespace m2::sic_robot
