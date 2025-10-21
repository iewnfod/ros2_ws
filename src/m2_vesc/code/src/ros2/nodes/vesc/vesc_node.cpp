#include "m2_vesc/ros2/nodes/vesc_node/vesc_node.hpp"
#include "m2_vesc/comm/vesc_socketcan_comm.hpp"

#include "trolly/log/trolly_logger_macro.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

namespace m2::vesc {

vesc_node::vesc_node() noexcept
    : Node("vesc_node")
{
    declare_parameter("ifr_name", "vcan0");
    declare_parameter("vesc_id", 1);
}

void vesc_node::init() noexcept
{
    auto ifr_name = get_parameter("ifr_name").as_string();
    auto vesc_id = get_parameter("vesc_id").as_int();

    vesc_ = std::make_unique<vesc_device>(std::make_unique<vesc_socketcan_comm>(ifr_name, vesc_id));
    vesc_->start();

    vel_pub_ = create_publisher<std_msgs::msg::Float32>("~/v_feedback", 1);
    pos_pub_ = create_publisher<std_msgs::msg::Float32>("~/p_feedback", 1);
    dc_pub_ = create_publisher<std_msgs::msg::Float32>("~/duty_cycle_feedback", 1);
    cur_pub_ = create_publisher<std_msgs::msg::Float32>("~/i_feedback", 10);

    vel_sub_ = create_subscription<std_msgs::msg::Float32>("~/v_setpoint", 1,
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const std_msgs::msg::Float32::SharedPtr msg) { vesc_->set_velocity(msg->data); });
    pos_sub_ = create_subscription<std_msgs::msg::Float32>("~/p_setpoint", 1,
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const std_msgs::msg::Float32::SharedPtr msg) { vesc_->set_position(msg->data); });
    dc_sub_ = create_subscription<std_msgs::msg::Float32>("~/duty_cycle_setpoint", 1,
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const std_msgs::msg::Float32::SharedPtr msg) { vesc_->set_duty_cycle(msg->data); });
    cur_sub_ = create_subscription<std_msgs::msg::Float32>("~/i_setpoint", 1,
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const std_msgs::msg::Float32::SharedPtr msg) { vesc_->set_current(msg->data); });

    vesc_->listen_velocity_feedback([this](float value) {
        std_msgs::msg::Float32 msg{};
        msg.data = value;
        vel_pub_->publish(msg);
    });
    vesc_->listen_position_feedback([this](float value) {
        std_msgs::msg::Float32 msg{};
        msg.data = value;
        pos_pub_->publish(msg);
    });
    vesc_->listen_duty_cycle_feedback([this](float value) {
        std_msgs::msg::Float32 msg{};
        msg.data = value;
        dc_pub_->publish(msg);
    });
    vesc_->listen_current_feedback([this](float value) {
        std_msgs::msg::Float32 msg{};
        msg.data = value;
        cur_pub_->publish(msg);
    });
}

void vesc_node::tick() noexcept { vesc_->tick(); }

void vesc_node::shutdown() noexcept { vesc_->stop(); }

}  // namespace m2::vesc

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<m2::vesc::vesc_node>();
    trolly::log::use_ros2_logger(node->get_logger(), node->get_clock());
    TROLLY_INFO("VESC node init start");
    node->init();
    TROLLY_INFO("VESC node init done");

    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
        node->tick();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    TROLLY_INFO("VESC node shutting down");
    node->shutdown();
    rclcpp::shutdown();
    return 0;
}
