#ifndef M2_SIC_ROBOT_NODES_FSM_FSM_NODE_H
#define M2_SIC_ROBOT_NODES_FSM_FSM_NODE_H

#include "m2_sic_robot/base/sic_robot.h"
#include "trolly/trolly_macro.h"

#include "rclcpp/rclcpp.hpp"

#include <string>

namespace m2::sic_robot {

class fsm_node : public rclcpp::Node {
public:
    fsm_node() noexcept;

    ~fsm_node() = default;

    bool init() noexcept;

    void run() noexcept;

private:
    int64_t loop_freq_{100};
    std::string input_topic_{"/input/joy_data"};
    std::unique_ptr<sic_robot> robot_;

    TROLLY_DISALLOW_COPY_AND_MOVE(fsm_node);
};

}  // namespace m2::sic_robot

#endif  // M2_SIC_ROBOT_NODES_FSM_FSM_NODE_H
