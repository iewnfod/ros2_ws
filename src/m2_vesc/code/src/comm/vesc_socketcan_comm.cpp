#include "m2_vesc/comm/vesc_socketcan_comm.hpp"

#include <trolly/log/trolly_logger_macro.h>

#include <net/if.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <span>

extern "C" {
#include <datatypes.h>
}

namespace m2::vesc {

static constexpr size_t STRERROR_BUFFER_LEN = 256;
static constexpr int32_t MAX_FD_COUNT = 32;
static constexpr uint32_t CAN_ID_MASK = 0x0FF;
static constexpr uint32_t TIMEOUT_MS = 1000;

// static constexpr float POSITION_SET_SCALE = 1000000.0F;
static constexpr float POSITION_FEEDBACK_SCALE = 50.0F;
static constexpr float CURRENT_SET_SCALE = 1000.0F;
static constexpr float CURRENT_FEEDBACK_SCALE = 10.0F;
static constexpr float DUTY_CYCLE_SET_SCALE = 100000.0F;
static constexpr float DUTY_CYCLE_FEEDBACK_SCALE = 1000.0F;

// TODO(Anthony Law): Switchable protocols
static constexpr uint32_t PROPRIETARY_SET_NORMAL_POS = 0x00000043;
static constexpr uint32_t PROPRIETARY_STATUS_6 = 0x00000060;
static constexpr float PROPRIETARY_POSITION_SET_SCALE = 50.0F;

namespace {

// TODO(Anthony Law): Switchable protocols
// void encode_position(std::span<uint8_t, 4> data, float value) noexcept
// {
//     auto val_int = static_cast<int32_t>(value * POSITION_SET_SCALE);
//     data[0] = static_cast<uint8_t>((val_int >> 24) & 0xFF);
//     data[1] = static_cast<uint8_t>((val_int >> 16) & 0xFF);
//     data[2] = static_cast<uint8_t>((val_int >> 8) & 0xFF);
//     data[3] = static_cast<uint8_t>(val_int & 0xFF);
// }

void encode_proprietary_position(std::span<uint8_t, 4> data, float value) noexcept
{
    auto val_int = static_cast<int32_t>(value * PROPRIETARY_POSITION_SET_SCALE);
    data[0] = static_cast<uint8_t>((val_int >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((val_int >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((val_int >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(val_int & 0xFF);
}

void encode_velocity(std::span<uint8_t, 4> data, float value) noexcept
{
    auto val_int = static_cast<int32_t>(value);
    data[0] = static_cast<uint8_t>((val_int >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((val_int >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((val_int >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(val_int & 0xFF);
}

void encode_current(std::span<uint8_t, 4> data, float value) noexcept
{
    auto val_int = static_cast<int32_t>(value * CURRENT_SET_SCALE);
    data[0] = static_cast<uint8_t>((val_int >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((val_int >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((val_int >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(val_int & 0xFF);
}

void encode_duty_cycle(std::span<uint8_t, 4> data, float value) noexcept
{
    auto val_int = static_cast<int32_t>(value * DUTY_CYCLE_SET_SCALE);
    data[0] = static_cast<uint8_t>((val_int >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((val_int >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((val_int >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(val_int & 0xFF);
}

void decode_status_1(
    std::span<const uint8_t, 8> data, float& rpm, float& current, float& duty_cycle) noexcept
{
    int32_t rpm_int = 0;
    rpm_int = data[0] << 24U;
    rpm_int |= data[1] << 16U;
    rpm_int |= data[2] << 8U;
    rpm_int |= data[3] & 0xFF;
    rpm = static_cast<float>(rpm_int);

    int32_t current_int = 0;
    current_int |= data[4] << 8U;
    current_int |= data[5] & 0xFF;
    current = static_cast<float>(current_int) / CURRENT_FEEDBACK_SCALE;

    int32_t duty_cycle_int = 0;
    duty_cycle_int |= data[6] << 8U;
    duty_cycle_int |= data[7] & 0xFF;
    duty_cycle = static_cast<float>(duty_cycle_int) / DUTY_CYCLE_FEEDBACK_SCALE;
}

// void decode_status_4(std::span<const uint8_t, 8> data, float& pos) noexcept
// {
//     int32_t pos_int = 0;
//     pos_int |= data[6] << 8U;
//     pos_int |= data[7] & 0xFF;
//     pos = static_cast<float>(pos_int) / POSITION_FEEDBACK_SCALE;
// }

void decode_proprietary_status_6(std::span<const uint8_t, 8> data, float& pos) noexcept
{
    int32_t pos_int = 0;
    pos_int |= data[0] << 24U;
    pos_int |= data[1] << 16U;
    pos_int |= data[2] << 8U;
    pos_int |= data[3] & 0xFF;
    pos = static_cast<float>(pos_int) / POSITION_FEEDBACK_SCALE;
}

void epoll_wait_events(fd_t event_fd, std::vector<fd_t>& fds)
{
    epoll_event events[MAX_FD_COUNT];

    auto cnt = epoll_wait(event_fd, events, MAX_FD_COUNT, -1);
    if (cnt <= 0) {
        return;
    }

    const int32_t max_event_num = cnt > MAX_FD_COUNT ? MAX_FD_COUNT : cnt;
    for (int32_t idx = 0; idx < max_event_num; ++idx) {
        if ((events[idx].events & EPOLLIN) != 0U) {
            fds.push_back(events[idx].data.fd);
        }
    }
}

}  // namespace

vesc_socketcan_comm::vesc_socketcan_comm(
    const std::string& interface_name, uint32_t vesc_id) noexcept
    : interface_name_(interface_name)
    , vesc_id_(vesc_id)
{}

vesc_err_t vesc_socketcan_comm::start() noexcept
{
    if (running_) {
        return vesc_err_t::TROLLY_ERR_INVALID_STATE;
    }

    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd_ < 0) {
        TROLLY_ERROR("Failed to open socket for CAN");
        return vesc_err_t::TROLLY_ERR_FAILURE;
    }

    ifreq ifr{};
    auto curr_len = interface_name_.size();
    const size_t copy_len = curr_len > IF_NAMESIZE ? IF_NAMESIZE : curr_len;
    interface_name_.copy(ifr.ifr_name, copy_len);
    ioctl(sock_fd_, SIOCGIFINDEX, &ifr);

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    if (bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        TROLLY_ERROR("Failed to bind the socket to CAN");
        return vesc_err_t::TROLLY_ERR_FAILURE;
    }
    TROLLY_ERROR("Binded socket to CAN successfully, ifr_name = %s", interface_name_.c_str());

    epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
    {
        epoll_event event{};
        event.events = EPOLLIN;
        event.data.fd = sock_fd_;

        const auto err = epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, sock_fd_, &event);
        if (err < 0) {
            char buffer[STRERROR_BUFFER_LEN];
            strerror_r(err, buffer, STRERROR_BUFFER_LEN);
            TROLLY_ERROR("Cannot open read event: %s", buffer);
            return vesc_err_t::TROLLY_ERR_FAILURE;
        }
    }

    exit_fd_ = eventfd(0, EFD_SEMAPHORE);
    {
        epoll_event event{};
        event.data.fd = exit_fd_;
        event.events = EPOLLIN | EPOLLET;

        const auto err = epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, exit_fd_, &event);
        if (err < 0) {
            char buffer[STRERROR_BUFFER_LEN];
            strerror_r(err, buffer, STRERROR_BUFFER_LEN);
            TROLLY_ERROR("Cannot open exit event: %s", buffer);
            return vesc_err_t::TROLLY_ERR_FAILURE;
        }
    }

    can_filter rfilters[1];

    const uint8_t rfilter_idx = 0;
    rfilters[rfilter_idx].can_id = vesc_id_;
    rfilters[rfilter_idx].can_mask = CAN_ID_MASK;

    if (setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilters, sizeof(rfilters)) != 0) {
        TROLLY_ERROR("Cannot set CAN filter");
        return vesc_err_t::TROLLY_ERR_FAILURE;
    }

    running_ = true;
    last_resp_time_ = std::chrono::steady_clock::now();
    recv_thread_ = std::thread([this] { recv_thread_routine(); });
    return vesc_err_t::TROLLY_ERR_SUCCESS;
}

vesc_err_t vesc_socketcan_comm::stop() noexcept
{
    if (!running_) {
        return vesc_err_t::TROLLY_ERR_INVALID_STATE;
    }
    running_ = false;
    return vesc_err_t::TROLLY_ERR_SUCCESS;
}

void vesc_socketcan_comm::recv_thread_routine() noexcept
{
    std::vector<fd_t> event_fds;
    while (running_) {
        epoll_wait_events(epoll_fd_, event_fds);
        for (const auto& event_fd : event_fds) {
            if (event_fd == exit_fd_) {
                return;
            }
            if (event_fd != sock_fd_) {
                continue;
            }

            can_frame frame{};
            auto nbytes = read(event_fd, &frame, sizeof(can_frame));

            if (nbytes < 0) {
                TROLLY_WARN("Nothing received from SocketCAN");
                continue;
            }

            if (static_cast<size_t>(nbytes) < sizeof(can_frame)) {
                TROLLY_WARN("Nothing incomplete can frame received");
                continue;
            }

            handle_can_frame(frame);
        }
    }
}

void vesc_socketcan_comm::handle_can_frame(const can_frame& frame) noexcept
{
    {
        const std::scoped_lock lock{last_resp_time_mutex_};
        last_resp_time_ = std::chrono::steady_clock::now();
    }
    const uint8_t status_id = (frame.can_id >> 8) & 0xFF;

    if (status_id == CAN_PACKET_STATUS) {
        float rpm = 0.0F;
        float current = 0.0F;
        float duty_cycle = 0.0F;
        decode_status_1(std::span{frame.data}, rpm, current, duty_cycle);
        notify_velocity_feedback(rpm);
        notify_current_feedback(current);
        notify_duty_cycle_feedback(duty_cycle);
        // } else if (status_id == CAN_PACKET_STATUS_4) {
        //     float pos = 0.0F;
        //     decode_status_4(std::span{frame.data}, pos);
        //     notify_position_feedback(pos);
    } else if (status_id == PROPRIETARY_STATUS_6) {
        // TODO(Anthony Law): Switchable protocols
        float pos = 0.0F;
        decode_proprietary_status_6(std::span{frame.data}, pos);
        notify_position_feedback(pos);
    }
}

vesc_err_t vesc_socketcan_comm::send_can_frame(
    uint32_t cmd_id, const uint8_t* data, size_t size) const noexcept
{
    can_frame frame{};
    frame.can_id = cmd_id << 8U | vesc_id_ | CAN_EFF_FLAG;
    // can_dlc is deprecated but len is not available in some older kernel
    frame.can_dlc = size;

    auto len = size > CAN_MAX_DLEN ? CAN_MAX_DLEN : size;
    std::copy(data, data + len, frame.data);

    const auto err = write(sock_fd_, &frame, sizeof(frame));
    if (err < 0) {
        TROLLY_WARN("Failed to write socket");
        return vesc_err_t::TROLLY_ERR_FAILURE;
    }

    return vesc_err_t::TROLLY_ERR_SUCCESS;
}

bool vesc_socketcan_comm::timed_out() noexcept
{
    const std::scoped_lock lock{last_resp_time_mutex_};
    return std::chrono::steady_clock::now() - last_resp_time_
        > std::chrono::milliseconds(TIMEOUT_MS);
}

vesc_err_t vesc_socketcan_comm::send_ping() noexcept
{
    return send_can_frame(CAN_PACKET_PING, nullptr, 0);
}

vesc_err_t vesc_socketcan_comm::send_position(float value) noexcept
{
    uint8_t data[4];
    // encode_position(std::span{data}, value);
    // return send_can_frame(CAN_PACKET_SET_POS, data, sizeof(data));
    encode_proprietary_position(std::span{data}, value);
    return send_can_frame(PROPRIETARY_SET_NORMAL_POS, data, sizeof(data));
}

vesc_err_t vesc_socketcan_comm::send_velocity(float value) noexcept
{
    uint8_t data[4];
    encode_velocity(std::span{data}, value);
    return send_can_frame(CAN_PACKET_SET_RPM, data, sizeof(data));
}

vesc_err_t vesc_socketcan_comm::send_current(float value) noexcept
{
    uint8_t data[4];
    encode_current(std::span{data}, value);
    return send_can_frame(CAN_PACKET_SET_CURRENT, data, sizeof(data));
}

vesc_err_t vesc_socketcan_comm::send_duty_cycle(float value) noexcept
{
    uint8_t data[4];
    encode_duty_cycle(std::span{data}, value);
    return send_can_frame(CAN_PACKET_SET_DUTY, data, sizeof(data));
}

}  // namespace m2::vesc
