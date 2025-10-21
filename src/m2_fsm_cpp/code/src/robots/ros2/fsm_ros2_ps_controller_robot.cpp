#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_controller_robot.h"

namespace m2::fsm {

ros2_ps_controller_robot::ros2_ps_controller_robot(std::shared_ptr<rclcpp::Node> node, uint32_t hz,
    const std::string& input_topic, input_handler_store_type& input_handlers) noexcept
    : ros2_robot(std::move(node), hz)
    , input_topic_(input_topic)
{
    for (auto& handler : input_handlers) {
        input_handlers_.emplace_back(std::move(handler));
    }
}

void ros2_ps_controller_robot::initialize() noexcept
{
    input_sub_ = node_handle()->create_subscription<ps_input_data_type>(input_topic_,
        rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
        [this](const ps_input_data_type::ConstSharedPtr& msg) { this->input_callback(msg); });
}

void ros2_ps_controller_robot::input_callback(const ps_input_data_type::ConstSharedPtr& msg)
{
    for (auto& handler : input_handlers_) {
        handler->input_callback(msg);
    }
}

}  // namespace m2::fsm
