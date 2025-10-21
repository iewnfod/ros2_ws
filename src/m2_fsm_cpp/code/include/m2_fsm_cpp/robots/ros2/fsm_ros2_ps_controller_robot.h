#ifndef M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_CONTROLLER_ROBOT_H
#define M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_CONTROLLER_ROBOT_H

#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_input.h"
#include "m2_fsm_cpp/robots/ros2/fsm_ros2_robot.h"

namespace m2::fsm {

class ros2_ps_controller_robot : public ros2_robot {
public:
    using input_handler_store_type = std::vector<input::ps_input_handler_ptr>;

    explicit ros2_ps_controller_robot(std::shared_ptr<rclcpp::Node> node, uint32_t hz,
        const std::string& input_topic, input_handler_store_type& input_handlers) noexcept;
    ~ros2_ps_controller_robot() noexcept override = default;

    void initialize() noexcept override;

private:
    using ps_input_data_type = m2_interfaces::msg::JoyData;
    using input_callback_func_type = void (ros2_ps_controller_robot::*)(
        const ps_input_data_type::ConstSharedPtr&);
    void input_callback(const ps_input_data_type::ConstSharedPtr& msg);

private:
    input_handler_store_type input_handlers_;
    rclcpp::Subscription<ps_input_data_type>::SharedPtr input_sub_;
    const std::string input_topic_;

    TROLLY_DISALLOW_COPY_AND_MOVE(ros2_ps_controller_robot);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_CONTROLLER_ROBOT_H
