#ifndef M2_SIC_ROBOT_SUBSYSTEMS_GRIPPER_SUBSYSTEM_H
#define M2_SIC_ROBOT_SUBSYSTEMS_GRIPPER_SUBSYSTEM_H

#include "m2_fsm_cpp/base/fsm_subsystem.h"
#include "m2_sic_robot/base/sic_robot_define.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include <gpiod.hpp>

namespace m2::sic_robot {

class gripper_subsystem : public fsm::subsystem {
public:
    explicit gripper_subsystem(std::shared_ptr<rclcpp::Node> node) noexcept;
    ~gripper_subsystem() noexcept = default;

    void tick() noexcept override;
	void set_elevator_front(bool up) noexcept;
	void set_elevator_back(bool up) noexcept;
	void set_gripper(bool up) noexcept;

private:
    void gripper_sub_cb(const std_msgs::msg::Bool& msg) noexcept;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;

	// gpiod::line right_;
	// gpiod::line elevator_;
    gpiod::line elevator_front_;
    gpiod::line elevator_back_;
    gpiod::line gripper_;

    TROLLY_DISALLOW_COPY_AND_MOVE(gripper_subsystem);
};

}  // namespace m2::sic_robot

#endif  // M2_SIC_ROBOT_SUBSYSTEMS_GRIPPER_SUBSYSTEM_H
