#include "trolly/math/trolly_math_util.h"

namespace trolly::math {

double l2_norm(const vector_t& vec) { return std::sqrt(vec.x * vec.x + vec.y * vec.y); }

double l2_norm(const point_t& point_a, const point_t& point_b)
{
    return l2_norm(point_a - point_b);
}

vector_t normalize(const vector_t& vec)
{
    auto mag = l2_norm(vec);
    if (mag == 0) {
        return vec;
    }
    return {vec.x / mag, vec.y / mag};
}

double dot_product(const vector_t& lhs, const vector_t& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

bool collinear(const point_t& a, const point_t& b, const point_t& c, double threshold)
{
    auto area =
        std::abs(a.x * b.y + b.x * c.y + c.x * a.y - (b.x * a.y - c.x * b.y - a.x * c.y)) / 2;
    return area < threshold;
}

euler_angles_t quaternion_to_euler(const quaternion_t& quaternion)
{
    // Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
    euler_angles_t angles{};

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
    double cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
    if (std::abs(sinp) >= 1) {
        angles.pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    } else {
        angles.pitch = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    double cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

quaternion_t euler_to_quaternion(const euler_angles_t& euler_angles)
{
    // Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code
    double cr = std::cos(euler_angles.roll * 0.5);
    double sr = std::sin(euler_angles.roll * 0.5);
    double cp = std::cos(euler_angles.pitch * 0.5);
    double sp = std::sin(euler_angles.pitch * 0.5);
    double cy = std::cos(euler_angles.yaw * 0.5);
    double sy = std::sin(euler_angles.yaw * 0.5);

    quaternion_t q{};
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

double normalize_angle(double angle)
{
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0) {
        return result + M_PI;
    }
    return result - M_PI;
}

double wrapped_angle_difference(double from, double to) { return normalize_angle(to - from); }

bool is_close(double x, double y, double threshold) { return fabs(x - y) <= threshold; }

}  // namespace trolly::math
