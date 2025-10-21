/**
 * @file vesc_comm.hpp
 * @author Anthony Law (anthlaw@connect.hku.hk)
 * @brief VESC Communication Interface
 * @version 0.1
 * @date 2024-09-18
 * 
 * @copyright Copyright (c) 2024 HKU Robocon Team
 * 
 */

#ifndef VESC_COMM_HPP
#define VESC_COMM_HPP

#include "m2_vesc/vesc_fundamental.hpp"

#include <trolly/trolly_errno.h>
#include <trolly/trolly_macro.h>

#include <functional>
#include <vector>

namespace m2::vesc {

class vesc_comm {
public:
    using feedback_func_type = std::function<void(float)>;

    vesc_comm() noexcept = default;

    virtual ~vesc_comm() noexcept = default;

    virtual vesc_err_t start() noexcept = 0;

    virtual vesc_err_t stop() noexcept = 0;

    virtual bool timed_out() noexcept = 0;

    virtual vesc_err_t send_ping() noexcept = 0;

    virtual vesc_err_t send_position(float value) noexcept = 0;

    virtual vesc_err_t send_velocity(float value) noexcept = 0;

    virtual vesc_err_t send_current(float value) noexcept = 0;

    virtual vesc_err_t send_duty_cycle(float value) noexcept = 0;

    void listen_position_feedback(feedback_func_type&& func) noexcept;

    void listen_velocity_feedback(feedback_func_type&& func) noexcept;

    void listen_current_feedback(feedback_func_type&& func) noexcept;

    void listen_duty_cycle_feedback(feedback_func_type&& func) noexcept;

protected:
    void notify_position_feedback(float value) noexcept;

    void notify_velocity_feedback(float value) noexcept;

    void notify_current_feedback(float value) noexcept;

    void notify_duty_cycle_feedback(float value) noexcept;

private:
    std::vector<feedback_func_type> position_feedback_chain_;
    std::vector<feedback_func_type> velocity_feedback_chain_;
    std::vector<feedback_func_type> current_feedback_chain_;
    std::vector<feedback_func_type> duty_cycle_feedback_chain_;

    TROLLY_DISALLOW_COPY_AND_MOVE(vesc_comm);
};

}  // namespace m2::vesc

#endif  // VESC_COMM_HPP
