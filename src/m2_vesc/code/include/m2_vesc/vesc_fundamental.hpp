/**
 * @file vesc_fundamental.hpp
 * @author Anthony Law (anthlaw@connect.hku.hk)
 * @brief VESC Fundamentals
 * @version 0.1
 * @date 2024-09-18
 * 
 * @copyright Copyright (c) 2024 HKU Robocon Team
 * 
 */

#ifndef VESC_FUNDAMENTAL_HPP
#define VESC_FUNDAMENTAL_HPP

#include <trolly/trolly_errno.h>

namespace m2::vesc {

using vesc_err_t = trolly::err_t;
using fd_t = int;

}  // namespace m2::vesc

#endif  // VESC_FUNDAMENTAL_HPP
