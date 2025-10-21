#ifndef TROLLY_MATH_UTIL_H
#define TROLLY_MATH_UTIL_H

#include "trolly/math/trolly_math_define.h"

namespace trolly::math {

inline constexpr double IS_CLOSE_DEFAULT_THRESHOLD = 1e-6;

/**
 * @brief L2 norm of vector
 *
 * @param vec Vector
 *
 * @return L2 norm
 */
double l2_norm(const vector_t& vec);

/**
 * @brief L2 norm of vector, formed from two points
 *
 * @param point_a Point A
 * @param point_b Point B
 *
 * @return L2 norm of AB
 */
double l2_norm(const point_t& point_a, const point_t& point_b);

/**
 * @brief Normalize vector to unit length
 *
 * @param vec Vector
 *
 * @return Normalized vector
 */
vector_t normalize(const vector_t& vec);

/**
 * @brief Dot product of two vectors
 * @note u dot v = (length of projection of u on v) * (length of v)
 *               = (length of projection of v on u) * (length of u)
 *
 * @param lhs Vector u
 * @param rhs Vector v
 *
 * @return Dot product of u and v
 */
double dot_product(const vector_t& lhs, const vector_t& rhs);

/**
 * @brief Given 3 point, determine if the 3 points are collinear
 *
 * @param a Point A
 * @param b Point B
 * @param c Point C
 * @param threshold Threshold for determining collinearlity, smaller is stricter
 *
 * @return true - The 3 points are collinear
 */
bool collinear(const point_t& a, const point_t& b, const point_t& c, double threshold = 0.00001);

/**
 * @brief Given a quaternion, return the corresponding Euler angles
 *
 * @param quaternion Quaternion
 *
 * @return Euler angles
 */
euler_angles_t quaternion_to_euler(const quaternion_t& quaternion);

/**
 * @brief Given euler angles, return the corresponding Quaternion
 *
 * @param euler_angles Euler angles
 *
 * @return Quaternion
 */
quaternion_t euler_to_quaternion(const euler_angles_t& euler_angles);

/**
 * @brief Normalize input angle.
 *
 * @param angle Angle to be normalized
 *              Domain: any real number in radian
 *
 * @return Normalized angle
 *         Range: [-M_PI, M_PI]
 */
double normalize_angle(double angle);

/**
 * @brief Calculate shortest angular differences between two angles.
 *        Domain: any real number in radian
 *
 * @param from Angle from
 * @param to   Angle to
 *
 * @return Wrapped angle difference
 *         Range: [-M_PI, M_PI]
 */
double wrapped_angle_difference(double from, double to);

/**
 * @brief Check if two numbers are close enough.
 *        Domain: any real number
 *
 * @param x         first number
 * @param y         second number
 * @param threshold threshold for comparison
 *
 * @return Two numbers are close enough or not
 */
bool is_close(double x, double y, double threshold = IS_CLOSE_DEFAULT_THRESHOLD);

}  // namespace trolly::math

#endif
