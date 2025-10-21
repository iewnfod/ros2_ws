/**
 * @file vesc_socketcan_comm.hpp
 * @author Anthony Law (anthlaw@connect.hku.hk)
 * @brief VESC SocketCAN Communication
 * @version 0.1
 * @date 2024-09-18
 * 
 * @copyright Copyright (c) 2024 HKU Robocon Team
 * 
 */

#ifndef VESC_SOCKETCAN_COMM_HPP
#define VESC_SOCKETCAN_COMM_HPP

#include "m2_vesc/comm/vesc_comm.hpp"

#include <linux/can/raw.h>

#include <thread>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <mutex>

namespace m2::vesc {

class vesc_socketcan_comm : public vesc_comm {
public:
    vesc_socketcan_comm(const std::string& interface_name, uint32_t vesc_id) noexcept;

    ~vesc_socketcan_comm() noexcept override = default;

    vesc_err_t start() noexcept override;

    vesc_err_t stop() noexcept override;

    bool timed_out() noexcept override;

    vesc_err_t send_ping() noexcept override;

    vesc_err_t send_position(float value) noexcept override;

    vesc_err_t send_velocity(float value) noexcept override;

    vesc_err_t send_current(float value) noexcept override;

    vesc_err_t send_duty_cycle(float value) noexcept override;

private:
    void recv_thread_routine() noexcept;

    void handle_can_frame(const can_frame& frame) noexcept;

    vesc_err_t send_can_frame(uint32_t cmd_id, const uint8_t* data, size_t size) const noexcept;

private:
    std::string interface_name_;
    uint32_t vesc_id_;
    fd_t sock_fd_{-1};
    fd_t epoll_fd_{-1};
    fd_t exit_fd_{-1};
    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    std::mutex last_resp_time_mutex_;
    std::chrono::steady_clock::time_point last_resp_time_;

    TROLLY_DISALLOW_COPY_AND_MOVE(vesc_socketcan_comm);
};

}  // namespace m2::vesc

#endif  // VESC_SOCKETCAN_COMM_HPP
