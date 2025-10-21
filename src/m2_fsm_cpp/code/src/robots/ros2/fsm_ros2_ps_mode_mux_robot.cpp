#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_mode_mux_robot.h"

#include "trolly/log/trolly_logger_macro.h"

using RgbTime = m2_interfaces::msg::JoyRgbTime;
using SetRgb = m2_interfaces::srv::JoySetRgb;

namespace m2::fsm {

ros2_ps_mode_mux_robot::ros2_ps_mode_mux_robot(
    std::shared_ptr<rclcpp::Node> node, uint32_t hz, const std::string& input_topic) noexcept
    : ros2_robot(std::move(node), hz)
    , input_topic_(input_topic)
{
    node_handle()->declare_parameter("set_led_srv", "/input/set_joy_led");

    input_sub_ = node_handle()->create_subscription<ps_input_data_type>(input_topic_,
        rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
        [this](const ps_input_data_type::ConstSharedPtr& msg) { this->input_callback(msg); });
    set_led_client_ = node_handle()->create_client<SetRgb>(
        node_handle()->get_parameter("set_led_srv").as_string());
    if (!set_led_client_->wait_for_service(std::chrono::seconds(10))) {
        set_led_srv_available_ = false;
        TROLLY_WARN("Joystick set LED service not available, functionality will be excluded");
    }
}

void ros2_ps_mode_mux_robot::initialize() noexcept { switch_mode(0); }

void ros2_ps_mode_mux_robot::tick() noexcept
{
    ros2_robot::tick();
    modes_[selected_mode_]->tick();
    for (auto& overlay : overlays_) {
        overlay->tick();
    }
}

void ros2_ps_mode_mux_robot::add_modes(mode_store_type& modes) noexcept
{
    for (auto& mode : modes) {
        modes_.emplace_back(std::move(mode));
    }
}

void ros2_ps_mode_mux_robot::add_overlays(mode_store_type& overlays) noexcept
{
    for (auto& overlay : overlays) {
        overlays_.emplace_back(std::move(overlay));
    }
}

void ros2_ps_mode_mux_robot::set_color_sequence(
    const input::ps_mode::color_sequence_type& color_sequence) noexcept
{
    if (TROLLY_UNLIKELY(!set_led_srv_available_)) {
        TROLLY_WARN(
            "[ps_mode_mux_robot] LED service not available, skipping color sequence update");
        return;
    }
    auto request = std::make_shared<set_rgb_srv_type::Request>();
    request->random = false;
    request->rgb_sequence = color_sequence;
    set_led_client_->async_send_request(request);
}

void ros2_ps_mode_mux_robot::switch_mode(uint32_t mode_idx) noexcept
{
    if (mode_idx >= modes_.size()) {
        return;
    }

    set_color_sequence(modes_[mode_idx]->color_sequence());

    selected_mode_ = mode_idx;
    TROLLY_WARN(
        "[ps_mode_mux_robot] Switching to mode %d: %s", mode_idx, modes_[mode_idx]->name().c_str());
    modes_[mode_idx]->on_enter();
}

void ros2_ps_mode_mux_robot::input_callback(const ps_input_data_type::ConstSharedPtr& msg) noexcept
{
    if (TROLLY_UNLIKELY(modes_.empty())) {
        return;
    }

    // switch mode
    if (msg->ps && !old_data_.ps) {
        modes_[selected_mode_]->on_exit();
        switch_mode((selected_mode_ + 1) % modes_.size());
    } else {
        // forward ps data to mode and overlays
        modes_[selected_mode_]->input_callback(msg);

        for (auto& overlay : overlays_) {
            overlay->input_callback(msg);
        }
    }

    old_data_ = *msg.get();
}

}  // namespace m2::fsm
