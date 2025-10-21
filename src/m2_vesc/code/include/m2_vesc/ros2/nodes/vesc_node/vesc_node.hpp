#ifndef VESC_NODE_HPP
#define VESC_NODE_HPP

#include "m2_vesc/vesc_device.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace m2::vesc {

class vesc_node : public rclcpp::Node {
public:
    using float32_pub_type = rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr;
    using float32_sub_type = rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr;

    vesc_node() noexcept;

    void init() noexcept;
    void tick() noexcept;
    void shutdown() noexcept;

private:
    std::unique_ptr<vesc_device> vesc_ = nullptr;
    float32_pub_type vel_pub_ = nullptr;
    float32_pub_type pos_pub_ = nullptr;
    float32_pub_type dc_pub_ = nullptr;
    float32_pub_type cur_pub_ = nullptr;
    float32_sub_type vel_sub_ = nullptr;
    float32_sub_type pos_sub_ = nullptr;
    float32_sub_type dc_sub_ = nullptr;
    float32_sub_type cur_sub_ = nullptr;
};

}  // namespace m2::vesc

#endif  // VESC_NODE_HPP
