#ifndef M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_MODE_H
#define M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_MODE_H

#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_input.h"
#include "m2_interfaces/msg/joy_rgb_time.hpp"

#include <string>

namespace m2::fsm::input {

class ps_mode;
using ps_mode_ptr = std::unique_ptr<ps_mode>;

class ps_mode : public ps_input_handler {
public:
    using joy_rgb_time_type = m2_interfaces::msg::JoyRgbTime;
    using color_sequence_type = std::vector<ps_mode::joy_rgb_time_type>;

    explicit ps_mode(const std::string& name) noexcept
        : name_(name)
        , color_sequence_(create_default_color_sequence("#FFFFFF"))
    {}
    explicit ps_mode(const std::string& name, const std::string& color_hex_str) noexcept
        : name_(name)
        , color_sequence_(create_default_color_sequence(color_hex_str))
    {}
    explicit ps_mode(const std::string& name, const color_sequence_type& color_sequence) noexcept
        : name_(name)
        , color_sequence_(color_sequence)
    {}
    ~ps_mode() noexcept override = default;

    TROLLY_NO_DISCARD_INLINE const std::string& name() const noexcept { return name_; }
    TROLLY_NO_DISCARD_INLINE const color_sequence_type& color_sequence() const noexcept
    {
        return color_sequence_;
    }
    virtual void tick() noexcept {}
    virtual void on_enter() noexcept = 0;
    virtual void on_exit() noexcept = 0;

    static joy_rgb_time_type create_rgb_time(const std::string& hex_str, float seconds) noexcept;
    static color_sequence_type create_default_color_sequence(const std::string& hex_str) noexcept;

private:
    std::string name_;
    color_sequence_type color_sequence_;

    TROLLY_DISALLOW_COPY_AND_MOVE(ps_mode);
};

}  // namespace m2::fsm::input

#endif  // M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_MODE_H
