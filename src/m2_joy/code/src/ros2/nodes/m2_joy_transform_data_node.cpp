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
    , controller_type_(ControllerType::UNKNOWN)
    , controller_detected_(false)
{
    auto input_topic = declare_parameter("input_topic", "/joy");
    auto output_topic = declare_parameter("joy_topic", "/input/joy_data");

    // Parameter to manually specify controller type (optional)
    auto controller_type_param = declare_parameter("controller_type", "auto");

    if (controller_type_param == "ps5" || controller_type_param == "ps4") {
        controller_type_ = ControllerType::PS5;
        controller_detected_ = true;
        TROLLY_INFO("Controller type manually set to PlayStation");
    } else if (controller_type_param == "xbox") {
        controller_type_ = ControllerType::XBOX;
        controller_detected_ = true;
        TROLLY_INFO("Controller type manually set to Xbox");
    } else {
        TROLLY_INFO("Controller type set to auto-detect");
    }

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

ControllerType TransformDataNode::detect_controller_type(const sensor_msgs::msg::Joy& joy) noexcept
{
    // Xbox controllers typically have 8 axes and 11 buttons (Xbox One/Series controllers)
    // PS4/PS5 controllers typically have 8 axes and 13-14 buttons
    // This is a heuristic-based detection

    size_t num_buttons = joy.buttons.size();
    size_t num_axes = joy.axes.size();

    TROLLY_INFO("Detecting controller: %zu buttons, %zu axes", num_buttons, num_axes);

    // Xbox controllers typically have 11 buttons
    // PS controllers typically have 13+ buttons
    if (num_buttons >= 13) {
        TROLLY_INFO("Detected PlayStation controller (13+ buttons)");
        return ControllerType::PS5;
    } else if (num_buttons >= 10 && num_buttons <= 12) {
        TROLLY_INFO("Detected Xbox controller (10-12 buttons)");
        return ControllerType::XBOX;
    } else {
        TROLLY_WARN(
            "Unknown controller type with %zu buttons, defaulting to PlayStation", num_buttons);
        return ControllerType::PS5;
    }
}

m2_interfaces::msg::JoyData TransformDataNode::transform_ps_controller(
    const sensor_msgs::msg::Joy& joy) noexcept
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

    return joy_msg;
}

m2_interfaces::msg::JoyData TransformDataNode::transform_xbox_controller(
    const sensor_msgs::msg::Joy& joy) noexcept
{
    m2_interfaces::msg::JoyData joy_msg;

    // Xbox controller axes mapping
    // axes[0] = Left stick X (left = +1.0, right = -1.0)
    // axes[1] = Left stick Y (up = +1.0, down = -1.0)
    // axes[2] = LT analog (not pressed = 1.0, fully pressed = -1.0)
    // axes[3] = Right stick X
    // axes[4] = Right stick Y
    // axes[5] = RT analog (not pressed = 1.0, fully pressed = -1.0)
    // axes[6] = D-pad X
    // axes[7] = D-pad Y

    joy_msg.hat_lx = joy.axes[0];
    joy_msg.hat_ly = joy.axes[1];
    joy_msg.l2_analog = (joy.axes.size() > 2) ? (-(joy.axes[2] - 1) / 2) : 0;
    joy_msg.hat_rx = (joy.axes.size() > 3) ? joy.axes[3] : 0;
    joy_msg.hat_ry = (joy.axes.size() > 4) ? joy.axes[4] : 0;
    joy_msg.r2_analog = (joy.axes.size() > 5) ? (-(joy.axes[5] - 1) / 2) : 0;
    joy_msg.dpad_x = (joy.axes.size() > 6) ? joy.axes[6] : 0;
    joy_msg.dpad_y = (joy.axes.size() > 7) ? joy.axes[7] : 0;

    // Xbox controller button mapping (xpad driver)
    // buttons[0] = A (maps to PS Cross)
    // buttons[1] = B (maps to PS Circle)
    // buttons[2] = X (maps to PS Square)
    // buttons[3] = Y (maps to PS Triangle)
    // buttons[4] = LB (maps to PS L1)
    // buttons[5] = RB (maps to PS R1)
    // buttons[6] = View/Back (maps to PS Share)
    // buttons[7] = Menu/Start (maps to PS Options)
    // buttons[8] = Xbox/Guide (maps to PS button)
    // buttons[9] = Left stick press (L3)
    // buttons[10] = Right stick press (R3)

    joy_msg.cross = joy.buttons[0];                                        // A button
    joy_msg.circle = joy.buttons[1];                                       // B button
    joy_msg.square = (joy.buttons.size() > 2) ? joy.buttons[2] : false;    // X button
    joy_msg.triangle = (joy.buttons.size() > 3) ? joy.buttons[3] : false;  // Y button
    joy_msg.l1 = (joy.buttons.size() > 4) ? joy.buttons[4] : false;        // LB
    joy_msg.r1 = (joy.buttons.size() > 5) ? joy.buttons[5] : false;        // RB

    // LT and RT are typically axes, but we set the button state based on threshold
    joy_msg.l2 = joy_msg.l2_analog > 0.1;
    joy_msg.r2 = joy_msg.r2_analog > 0.1;

    joy_msg.share = (joy.buttons.size() > 6) ? joy.buttons[6] : false;    // View/Back
    joy_msg.options = (joy.buttons.size() > 7) ? joy.buttons[7] : false;  // Menu/Start
    joy_msg.ps = (joy.buttons.size() > 8) ? joy.buttons[8] : false;       // Xbox/Guide
    joy_msg.l3 = (joy.buttons.size() > 9) ? joy.buttons[9] : false;       // Left stick press
    joy_msg.r3 = (joy.buttons.size() > 10) ? joy.buttons[10] : false;     // Right stick press

    return joy_msg;
}

void TransformDataNode::joy_cb(const sensor_msgs::msg::Joy& joy) noexcept
{
    // Auto-detect controller type on first message if not manually specified
    if (!controller_detected_) {
        controller_type_ = detect_controller_type(joy);
        controller_detected_ = true;
    }

    // Transform based on controller type
    if (controller_type_ == ControllerType::XBOX) {
        last_msg_ = transform_xbox_controller(joy);
    } else {
        // Default to PS controller (PS4/PS5)
        last_msg_ = transform_ps_controller(joy);
    }
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
