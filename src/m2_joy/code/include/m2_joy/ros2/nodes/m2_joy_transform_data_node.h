#ifndef M2_JOY_TRANSFORM_DATA_NODE_H
#define M2_JOY_TRANSFORM_DATA_NODE_H

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "m2_interfaces/msg/joy_data.hpp"

namespace m2 {

class TransformDataNode : public rclcpp::Node {
public:
    TransformDataNode() noexcept;

private:
    std::optional<m2_interfaces::msg::JoyData> last_msg_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<m2_interfaces::msg::JoyData>::SharedPtr joy_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    static bool check_kernel_version() noexcept;

    void joy_cb(const sensor_msgs::msg::Joy& joy) noexcept;
};

}  // namespace m2

#endif  // M2_JOY_TRANSFORM_DATA_NODE_H
