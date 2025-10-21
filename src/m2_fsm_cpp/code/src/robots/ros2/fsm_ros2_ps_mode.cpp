#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_mode.h"

namespace m2::fsm::input {

namespace {

inline uint8_t parse_hex_byte(const std::string& hex)
{
    return static_cast<uint8_t>(std::stoi(hex, nullptr, 16));
}

}  // namespace

ps_mode::joy_rgb_time_type ps_mode::create_rgb_time(
    const std::string& hex_str, float seconds) noexcept
{
    size_t offset = 0;
    if (hex_str.starts_with('#')) {
        offset++;
    }
    joy_rgb_time_type msg;
    msg.red = parse_hex_byte(hex_str.substr(offset + 0, offset + 2));
    msg.green = parse_hex_byte(hex_str.substr(offset + 2, offset + 2));
    msg.blue = parse_hex_byte(hex_str.substr(offset + 4, offset + 2));
    msg.seconds = seconds;
    return msg;
}

ps_mode::color_sequence_type ps_mode::create_default_color_sequence(
    const std::string& hex_str) noexcept
{
    return {
        create_rgb_time(hex_str, 1.0F),
        create_rgb_time("#000000", 0.1F),
    };
}

}  // namespace m2::fsm::input
