#include "m2_joy/ros2/nodes/m2_joy_transform_data_node.h"

#include <sys/utsname.h>
#include <chrono>
#include <memory>
#include <string>

#include "trolly/log/trolly_logger_macro.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

namespace m2 {

TransformDataNode::TransformDataNode() noexcept
    : Node("joy_transform_data_node")
{
    auto input_topic = declare_parameter("input_topic", "/joy");
    auto output_topic = declare_parameter("joy_topic", "/input/joy_data");

    joy_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
        input_topic, 1, [this](const sensor_msgs::msg::Joy& msg) { this->joy_cb(msg); });
    joy_publisher_ = create_publisher<m2_interfaces::msg::JoyData>(output_topic, 1);

    if (!check_kernel_version()) {
        TROLLY_WARN("Kernel version is not supported. Please use kernel version 5.3 or higher.");
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
        if (last_msg_.has_value()) {
            this->joy_publisher_->publish(last_msg_.value());
        }
    });
}

bool TransformDataNode::check_kernel_version() noexcept
{
    utsname buffer;
    if (uname(&buffer) != 0) {
        perror("uname");
        return false;
    }
    return (buffer.release[0] > '5' || (buffer.release[0] == '5' && buffer.release[3] >= '3'));
}

void TransformDataNode::joy_cb(const sensor_msgs::msg::Joy& joy) noexcept
{
    m2_interfaces::msg::JoyData joy_msg;
    joy_msg.hat_lx = joy.axes[0];
    joy_msg.hat_ly = joy.axes[1];
    joy_msg.l2_analog = joy.buttons[6] ? (-(joy.axes[2] - 1) / 2) : 0;
    joy_msg.hat_rx = joy.axes[3];
    joy_msg.hat_ry = joy.axes[4];
    joy_msg.r2_analog = joy.buttons[7] ? (-(joy.axes[5] - 1) / 2) : 0;
    joy_msg.dpad_x = joy.axes[6];
    joy_msg.dpad_y = joy.axes[7];
    joy_msg.cross = joy.buttons[0];
    joy_msg.circle = joy.buttons[1];
    joy_msg.triangle = joy.buttons[2];
    joy_msg.square = joy.buttons[3];
    joy_msg.l1 = joy.buttons[4];
    joy_msg.r1 = joy.buttons[5];
    joy_msg.l2 = joy.buttons[6];
    joy_msg.r2 = joy.buttons[7];
    joy_msg.share = joy.buttons[8];
    joy_msg.options = joy.buttons[9];
    joy_msg.ps = joy.buttons[10];
    joy_msg.l3 = joy.buttons[11];
    joy_msg.r3 = joy.buttons[12];

    last_msg_ = joy_msg;
}

}  // namespace m2

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<m2::TransformDataNode>();
    trolly::log::use_ros2_logger(node->get_logger(), node->get_clock());
    TROLLY_INFO("node initialized");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
