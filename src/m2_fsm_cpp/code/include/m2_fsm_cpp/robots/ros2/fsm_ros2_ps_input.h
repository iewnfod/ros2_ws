#ifndef M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_INPUT_HANDLER_H
#define M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_INPUT_HANDLER_H

#include "m2_interfaces/msg/joy_data.hpp"
#include "trolly/trolly_macro.h"

#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace m2::fsm::input {

enum class ps_analog_t {
    HAT_LX,
    HAT_LY,
    L2_ANALOG,
    HAT_RX,
    HAT_RY,
    R2_ANALOG,
};

enum class ps_dpad_direction_t {
    UP,
    DOWN,
    LEFT,
    RIGHT,
};

enum class ps_button_t {
    CROSS,
    CIRCLE,
    TRIANGLE,
    SQUARE,
    L1,
    R1,
    L2,
    R2,
    SHARE,
    OPTIONS,
    PS,
    L3,
    R3,
    TPAD_CLICK
};

class ps_input_handler;
using ps_input_handler_ptr = std::unique_ptr<ps_input_handler>;
class ps_input_handler {
public:
    using ps_input_data_type = m2_interfaces::msg::JoyData;
    using ps_input_data_const_pointer_type = typename ps_input_data_type::ConstSharedPtr;

    // gcc bug on not accepting default functions with noexcept, fixed in newer versions
    // NOLINTNEXTLINE(hicpp-use-equals-default, modernize-use-equals-default)
    explicit ps_input_handler() noexcept {}
    virtual ~ps_input_handler() noexcept = default;

    virtual void on_input(const ps_input_data_type& data) noexcept = 0;

    template<ps_button_t Button>
    TROLLY_NO_DISCARD bool is_button_pressed() const noexcept
    {
        return !get_button_value<Button>(old_data_)
            && (new_data_ptr_ == nullptr || get_button_value<Button>(*new_data_ptr_));
    }

    template<ps_button_t Button>
    TROLLY_NO_DISCARD bool is_button_released() const noexcept
    {
        return get_button_value<Button>(old_data_)
            && (new_data_ptr_ == nullptr || !get_button_value<Button>(*new_data_ptr_));
    }

    template<ps_button_t Button>
    TROLLY_NO_DISCARD bool is_button_pressing() const noexcept
    {
        return new_data_ptr_ != nullptr && get_button_value<Button>(*new_data_ptr_);
    }

    template<ps_dpad_direction_t DpadDirection>
    TROLLY_NO_DISCARD bool is_dpad_pressed() const noexcept
    {
        return !get_dpad_value<DpadDirection>(old_data_)
            && (new_data_ptr_ == nullptr || get_dpad_value<DpadDirection>(*new_data_ptr_));
    }

    template<ps_dpad_direction_t DpadDirection>
    TROLLY_NO_DISCARD bool is_dpad_released() const noexcept
    {
        return get_dpad_value<DpadDirection>(old_data_)
            && (new_data_ptr_ == nullptr || !get_dpad_value<DpadDirection>(*new_data_ptr_));
    }

    template<ps_dpad_direction_t DpadDirection>
    TROLLY_NO_DISCARD bool is_dpad_pressing() const noexcept
    {
        return new_data_ptr_ != nullptr && get_dpad_value<DpadDirection>(*new_data_ptr_);
    }

    template<ps_analog_t Analog>
    TROLLY_NO_DISCARD float get_analog_value() const noexcept
    {
        if (TROLLY_UNLIKELY(new_data_ptr_ == nullptr)) {
            return 0.0;
        }
        return get_analog_value_impl<Analog>(*new_data_ptr_);
    }

    template<ps_analog_t Analog>
    TROLLY_NO_DISCARD bool is_analog_value(float value) const noexcept
    {
        if (TROLLY_UNLIKELY(new_data_ptr_ == nullptr)) {
            return false;
        }
        return are_float_same(get_analog_value_impl<Analog>(*new_data_ptr_), value);
    }

    void input_callback(const m2_interfaces::msg::JoyData::ConstSharedPtr& msg) noexcept
    {
        new_data_ptr_ = msg;
        on_input(*new_data_ptr_);
        old_data_ = *new_data_ptr_;
    }

private:
    template<ps_button_t Button>
    static constexpr bool get_button_value(const ps_input_data_type& data) noexcept
    {
        if constexpr (Button == ps_button_t::CROSS) {
            return data.cross;
        } else if constexpr (Button == ps_button_t::CIRCLE) {
            return data.circle;
        } else if constexpr (Button == ps_button_t::TRIANGLE) {
            return data.triangle;
        } else if constexpr (Button == ps_button_t::SQUARE) {
            return data.square;
        } else if constexpr (Button == ps_button_t::L1) {
            return data.l1;
        } else if constexpr (Button == ps_button_t::R1) {
            return data.r1;
        } else if constexpr (Button == ps_button_t::L2) {
            return data.l2;
        } else if constexpr (Button == ps_button_t::R2) {
            return data.r2;
        } else if constexpr (Button == ps_button_t::SHARE) {
            return data.share;
        } else if constexpr (Button == ps_button_t::OPTIONS) {
            return data.options;
        } else if constexpr (Button == ps_button_t::PS) {
            return data.ps;
        } else if constexpr (Button == ps_button_t::L3) {
            return data.l3;
        } else if constexpr (Button == ps_button_t::R3) {
            return data.r3;
        } else if constexpr (Button == ps_button_t::TPAD_CLICK) {
            return data.tpad_click;
        }
        return false;
    }

    static bool are_float_same(float a, float b) noexcept
    {
        return fabsf(a - b) < std::numeric_limits<float>::epsilon();
    }

    template<ps_dpad_direction_t DpadDirection>
    static constexpr bool get_dpad_value(const ps_input_data_type& data) noexcept
    {
        if constexpr (DpadDirection == ps_dpad_direction_t::UP) {
            return are_float_same(data.dpad_y, 1.0F);
        } else if constexpr (DpadDirection == ps_dpad_direction_t::LEFT) {
            return are_float_same(data.dpad_x, 1.0F);
        } else if constexpr (DpadDirection == ps_dpad_direction_t::DOWN) {
            return are_float_same(data.dpad_y, -1.0F);
        } else if constexpr (DpadDirection == ps_dpad_direction_t::RIGHT) {
            return are_float_same(data.dpad_x, -1.0F);
        }
        return false;
    }

    template<ps_analog_t Analog>
    static constexpr float get_analog_value_impl(const ps_input_data_type& data) noexcept
    {
        if constexpr (Analog == ps_analog_t::HAT_LX) {
            return data.hat_lx;
        } else if constexpr (Analog == ps_analog_t::HAT_LY) {
            return data.hat_ly;
        } else if constexpr (Analog == ps_analog_t::L2_ANALOG) {
            return data.l2_analog;
        } else if constexpr (Analog == ps_analog_t::HAT_RX) {
            return data.hat_rx;
        } else if constexpr (Analog == ps_analog_t::HAT_RY) {
            return data.hat_ry;
        } else if constexpr (Analog == ps_analog_t::R2_ANALOG) {
            return data.r2_analog;
        }
        return false;
    }

private:
    ps_input_data_type old_data_;
    ps_input_data_const_pointer_type new_data_ptr_ = nullptr;

    TROLLY_DISALLOW_COPY_AND_MOVE(ps_input_handler);
};

}  // namespace m2::fsm::input

#endif  // M2_FSM_CPP_ROBOTS_ROS2_FSM_ROS2_PS_INPUT_HANDLER_H
