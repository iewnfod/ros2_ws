#ifndef TROLLY_MATH_DEFINE_H
#define TROLLY_MATH_DEFINE_H

#include "trolly/trolly_macro.h"

#ifdef TROLLY_COMPILER_MSVC
#define _USE_MATH_DEFINES
#endif

#include <cassert>
#include <cmath>

namespace trolly::math {

// 2D vector
struct vector_t {
    vector_t() = default;
    vector_t(double x, double y)
        : x(x)
        , y(y)
    {}

    vector_t operator+(const vector_t& other) const noexcept { return {x + other.x, y + other.y}; }

    vector_t operator-(const vector_t& other) const noexcept { return {x - other.x, y - other.y}; }

    vector_t operator*(double scale) const noexcept { return {x * scale, y * scale}; }

    vector_t operator/(double scale) const noexcept
    {
        assert(scale != 0);
        return {x / scale, y / scale};
    }

    bool operator==(const vector_t& rhs) const noexcept { return x == rhs.x && y == rhs.y; }

    bool operator!=(const vector_t& rhs) const noexcept { return !operator==(rhs); }

    double x{0.0};
    double y{0.0};
};

// Alias for more descriptive function parameter types
using point_t = vector_t;

struct quaternion_t {
    bool operator==(const quaternion_t& rhs) const noexcept
    {
        return (w == rhs.w) && (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
    }

    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct euler_angles_t {
    bool operator==(const euler_angles_t& rhs) const noexcept
    {
        return (roll == rhs.roll) && (pitch == rhs.pitch) && (yaw == rhs.yaw);
    }

    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

// A representation of pose in free space, composed of position and orientation
struct pose_t {
    vector_t position;
    double yaw{0.0};
};

}  // namespace trolly::math

#endif
