#include "m2_sic_robot/nodes/fsm/fsm_node.h"

#include "trolly/log/trolly_logger_macro.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

namespace m2::sic_robot {

fsm_node::fsm_node() noexcept
    : rclcpp::Node("fsm_node")
{
    declare_parameter<int64_t>("loop_freq", 100);
    declare_parameter<std::string>("input_topic", "/input/joy_data");
}

bool fsm_node::init() noexcept
{
    TROLLY_INFO("[fsm_node] Node init begin.");

    loop_freq_ = get_parameter("loop_freq").as_int();
    input_topic_ = get_parameter("input_topic").as_string();
    robot_ = std::make_unique<sic_robot>(shared_from_this(), loop_freq_, input_topic_);

    TROLLY_INFO("[fsm_node] Node init end.");
    return true;
}

void fsm_node::run() noexcept { robot_->start(); }

}  // namespace m2::sic_robot

void shutdown(int) {
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, shutdown);

    auto node = std::make_shared<m2::sic_robot::fsm_node>();
    trolly::log::use_ros2_logger(node->get_logger(), node->get_clock());

    if (node->init()) {
        node->run();
    } else {
        TROLLY_ERROR("[fsm_node] Node init fail! Exiting...");
    }

    rclcpp::shutdown();
    return 0;
}
