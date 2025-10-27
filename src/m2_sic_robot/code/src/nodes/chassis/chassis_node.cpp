#include "m2_sic_robot/nodes/chassis/chassis_node.h"
#include "m2_interfaces/msg/joy_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "trolly/log/trolly_logger_macro.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

#include <chrono>
#include <cstdlib>
#include <optional>

using namespace std::chrono_literals;

namespace m2::sic_robot {

chassis_node::chassis_node()
    : rclcpp::Node("chassis_node")
{
    max_vel_ = 40000.0;
    max_steer_differential_ = 15000.0;

    hat_ly_ = 0.0;
    hat_lx_ = 0.0;

    l_dead_zone_x_ = 0.2;
    l_dead_zone_y_ = 0.2;

    left_wheel_vel_ = 0.0;
    right_wheel_vel_ = 0.0;

    stop_flag = false;

    timer_ = this->create_wall_timer(
        10ms, [this](){this->tick();}
    );
}

bool chassis_node::init()
{
    TROLLY_INFO("[chassis_node] Node init begin.");

    controller_sub_ = this->create_subscription<m2_interfaces::msg::JoyData>(
        "/input/joy_data", 1,
        [this](const m2_interfaces::msg::JoyData &data){
            this->on_controller_data_cb(data);
        }
    );

    left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float32>("/left_wheel/v_setpoint", 1);
    right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float32>("/right_wheel/v_setpoint", 1);

    TROLLY_INFO("[chassis_node] Node init end.");
    return true;
}

void chassis_node::tick()
{
    if (stop_flag)
    {
        TROLLY_INFO("[chassis_node] Stop flag is set. Stopping the motor.");
        left_wheel_vel_ = 0.0;
        right_wheel_vel_ = 0.0;
    } else {
        TROLLY_INFO("[chassis_node] Controller - lx: %.2f, ly: %.2f", hat_lx_, hat_ly_);
        left_wheel_vel_ = hat_ly_ * max_vel_ - hat_lx_ * max_steer_differential_;
        right_wheel_vel_ = hat_ly_ * max_vel_ + hat_lx_ * max_steer_differential_;
    }

    std_msgs::msg::Float32 left_msg;
    std_msgs::msg::Float32 right_msg;

    left_msg.data = left_wheel_vel_;
    right_msg.data = right_wheel_vel_;

    left_wheel_pub_->publish(left_msg);
    right_wheel_pub_->publish(right_msg);

    TROLLY_INFO(
        "[chassis_node] Publish - left_wheel: %.2f/%.2f, right_wheel: %.2f/%.2f",
        left_wheel_vel_, max_vel_, right_wheel_vel_, max_vel_
    );
}

void chassis_node::on_controller_data_cb(const m2_interfaces::msg::JoyData& data)
{
    if (std::abs(data.hat_lx) > l_dead_zone_x_)
    {
        hat_lx_ = data.hat_lx;
    } else {
        hat_lx_ = 0.0;
    }

    if (std::abs(data.hat_ly) > l_dead_zone_y_)
    {
        hat_ly_ = data.hat_ly;
    } else {
        hat_ly_ = 0.0;
    }
}

}  // namespace m2::sic_robot

void shutdown(int)
{
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, shutdown);

    auto node = std::make_shared<m2::sic_robot::chassis_node>();
    trolly::log::use_ros2_logger(node->get_logger(), node->get_clock());

    if (node->init()) {
        rclcpp::spin(node);
    } else {
        TROLLY_ERROR("[chassis_node] Node init fail! Exiting...");
    }

    rclcpp::shutdown();
    return 0;
}
