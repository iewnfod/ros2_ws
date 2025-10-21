#ifndef M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_MODE_MUX_ROBOT_H
#define M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_MODE_MUX_ROBOT_H

#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_mode.h"
#include "m2_fsm_cpp/robots/ros2/fsm_ros2_robot.h"
#include "m2_interfaces/msg/joy_data.hpp"
#include "m2_interfaces/msg/joy_rgb_time.hpp"
#include "m2_interfaces/srv/joy_set_rgb.hpp"

#include <future>

namespace m2::fsm {

class ros2_ps_mode_mux_robot : public ros2_robot {
public:
    using mode_store_type = std::vector<input::ps_mode_ptr>;

    explicit ros2_ps_mode_mux_robot(
        std::shared_ptr<rclcpp::Node> node, uint32_t hz, const std::string& input_topic) noexcept;

    ~ros2_ps_mode_mux_robot() noexcept override = default;

    void initialize() noexcept override;
    void tick() noexcept override;

    void switch_mode(uint32_t mode_idx) noexcept;

protected:
    void add_modes(mode_store_type& modes) noexcept;
    void add_overlays(mode_store_type& overlays) noexcept;

private:
    using ps_input_data_type = m2_interfaces::msg::JoyData;
    using ps_input_data_sub_type = rclcpp::Subscription<ps_input_data_type>;
    using set_rgb_srv_type = m2_interfaces::srv::JoySetRgb;
    using set_rgb_client_type = rclcpp::Client<set_rgb_srv_type>;
    using input_callback_func_type = void (ros2_ps_mode_mux_robot::*)(
        const ps_input_data_type::ConstSharedPtr&);

    void input_callback(const ps_input_data_type::ConstSharedPtr& msg) noexcept;
    void set_color_sequence(const input::ps_mode::color_sequence_type& color_sequence) noexcept;

private:
    const std::string input_topic_;
    ps_input_data_type old_data_;
    mode_store_type modes_;
    mode_store_type overlays_;
    uint32_t selected_mode_{0};
    ps_input_data_sub_type::SharedPtr input_sub_;
    set_rgb_client_type::SharedPtr set_led_client_;
    bool set_led_srv_available_{true};

    TROLLY_DISALLOW_COPY_AND_MOVE(ros2_ps_mode_mux_robot);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_MODE_MUX_ROBOT_H
