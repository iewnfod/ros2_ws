#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "m2_interfaces/msg/joy_data.hpp"

#include "trolly/log/trolly_logger_macro.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

#include <chrono>

using namespace std::chrono_literals;

namespace m2::sic_robot{

class chassis_node : public rclcpp::Node {
public:
    explicit chassis_node();

    bool init();

    void tick();

    void on_controller_data_cb(const m2_interfaces::msg::JoyData& data);

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<m2_interfaces::msg::JoyData>::SharedPtr controller_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wheel_pub_;

    double max_vel_;  // max velocity
    double max_steer_differential_;

    double hat_ly_;  // controller left stick y
    double hat_lx_;  // controller left stick x

};

}
