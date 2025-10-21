/**
 * @file vesc_device.hpp
 * @author Anthony Law (anthlaw@connect.hku.hk)
 * @brief VESC Device
 * @version 0.1
 * @date 2024-09-18
 * 
 * @copyright Copyright (c) 2024 HKU Robocon Team
 * 
 */

#ifndef VESC_DEVICE_HPP
#define VESC_DEVICE_HPP

#include "m2_vesc/comm/vesc_comm.hpp"

#include <memory>

namespace m2::vesc {

class vesc_device {
public:
    using feedback_func_type = std::function<void(float)>;

    explicit vesc_device(std::unique_ptr<vesc_comm> comm) noexcept;

    ~vesc_device() noexcept = default;

    vesc_err_t start() noexcept;

    vesc_err_t stop() noexcept;

    vesc_err_t tick() noexcept;

    vesc_err_t set_position(float value) noexcept;

    vesc_err_t set_velocity(float value) noexcept;

    vesc_err_t set_current(float value) noexcept;

    vesc_err_t set_duty_cycle(float value) noexcept;

    void listen_position_feedback(feedback_func_type&& func) noexcept;

    void listen_velocity_feedback(feedback_func_type&& func) noexcept;

    void listen_current_feedback(feedback_func_type&& func) noexcept;

    void listen_duty_cycle_feedback(feedback_func_type&& func) noexcept;

private:
    std::unique_ptr<vesc_comm> comm_;
    bool running_{false};

    TROLLY_DISALLOW_COPY_AND_MOVE(vesc_device);
};

}  // namespace m2::vesc

#endif  // VESC_DEVICE_HPP
