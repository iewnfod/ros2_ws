#include "m2_vesc/comm/vesc_comm.hpp"

namespace m2::vesc {

void vesc_comm::listen_position_feedback(feedback_func_type&& func) noexcept
{
    position_feedback_chain_.emplace_back(std::move(func));
}

void vesc_comm::listen_velocity_feedback(feedback_func_type&& func) noexcept
{
    velocity_feedback_chain_.emplace_back(std::move(func));
}

void vesc_comm::listen_current_feedback(feedback_func_type&& func) noexcept
{
    current_feedback_chain_.emplace_back(std::move(func));
}

void vesc_comm::listen_duty_cycle_feedback(feedback_func_type&& func) noexcept
{
    duty_cycle_feedback_chain_.emplace_back(std::move(func));
}

void vesc_comm::notify_position_feedback(float value) noexcept
{
    for (auto& func : position_feedback_chain_) {
        func(value);
    }
}

void vesc_comm::notify_velocity_feedback(float value) noexcept
{
    for (auto& func : velocity_feedback_chain_) {
        func(value);
    }
}

void vesc_comm::notify_current_feedback(float value) noexcept
{
    for (auto& func : current_feedback_chain_) {
        func(value);
    }
}

void vesc_comm::notify_duty_cycle_feedback(float value) noexcept
{
    for (auto& func : duty_cycle_feedback_chain_) {
        func(value);
    }
}

}  // namespace m2::vesc
