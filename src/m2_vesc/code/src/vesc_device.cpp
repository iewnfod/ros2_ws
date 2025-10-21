#include "m2_vesc/vesc_device.hpp"

#include <trolly/log/trolly_logger_macro.h>

namespace m2::vesc {

vesc_device::vesc_device(std::unique_ptr<vesc_comm> comm) noexcept
    : comm_(std::move(comm))
{}

vesc_err_t vesc_device::start() noexcept
{
    if (running_) {
        return vesc_err_t::TROLLY_ERR_INVALID_STATE;
    }
    auto err = comm_->start();
    if (err != trolly::TROLLY_ERR_SUCCESS) {
        return err;
    }
    running_ = true;
    return vesc_err_t::TROLLY_ERR_SUCCESS;
}

vesc_err_t vesc_device::stop() noexcept
{
    if (!running_) {
        return vesc_err_t::TROLLY_ERR_INVALID_STATE;
    }
    auto err = comm_->stop();
    if (err != trolly::TROLLY_ERR_SUCCESS) {
        return err;
    }
    running_ = false;
    return vesc_err_t::TROLLY_ERR_SUCCESS;
}

vesc_err_t vesc_device::tick() noexcept
{
    if (!running_) {
        return vesc_err_t::TROLLY_ERR_INVALID_STATE;
    }
    if (comm_->timed_out()) {
        TROLLY_WARN_THROTTLE(1000, "VESC feedback timed out");
    } else {
        comm_->send_ping();
    }
    return vesc_err_t::TROLLY_ERR_SUCCESS;
}

vesc_err_t vesc_device::set_position(float value) noexcept { return comm_->send_position(value); }

vesc_err_t vesc_device::set_velocity(float value) noexcept { return comm_->send_velocity(value); }

vesc_err_t vesc_device::set_current(float value) noexcept { return comm_->send_current(value); }

vesc_err_t vesc_device::set_duty_cycle(float value) noexcept
{
    return comm_->send_duty_cycle(value);
}

void vesc_device::listen_position_feedback(feedback_func_type&& func) noexcept
{
    comm_->listen_position_feedback(std::move(func));
}

void vesc_device::listen_velocity_feedback(feedback_func_type&& func) noexcept
{
    comm_->listen_velocity_feedback(std::move(func));
}

void vesc_device::listen_current_feedback(feedback_func_type&& func) noexcept
{
    comm_->listen_current_feedback(std::move(func));
}

void vesc_device::listen_duty_cycle_feedback(feedback_func_type&& func) noexcept
{
    comm_->listen_duty_cycle_feedback(std::move(func));
}

}  // namespace m2::vesc
