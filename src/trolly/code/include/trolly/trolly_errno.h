/**
 * @file trolly_errno.h
 * @author Anthony Law (anthlaw@connect.hku.hk)
 * @brief Error numbers
 * @version 0.1
 * @date 2023-10-25
 * 
 * @copyright Copyright (c) 2023 HKU Robocon Team
 * 
 */

#ifndef TROLLY_ERRNO_H
#define TROLLY_ERRNO_H

#include <cstdint>
#include <string>

/// @cond
#define TROLLY_FOREACH_ERR(TROLLY_ERR)             \
    TROLLY_ERR(TROLLY_ERR_SUCCESS, 0x0000)         \
    TROLLY_ERR(TROLLY_ERR_FAILURE, 0xE001)         \
    TROLLY_ERR(TROLLY_ERR_TIMEOUT, 0xE002)         \
    TROLLY_ERR(TROLLY_ERR_UNSUPPORTED, 0xE003)     \
    TROLLY_ERR(TROLLY_ERR_NOT_IMPLEMENTED, 0xE004) \
    TROLLY_ERR(TROLLY_ERR_INVALID_STATE, 0xE005)

#define TROLLY_GENERATE_ENUM(ENUM, VALUE) ENUM = (VALUE),

#define TROLLY_GENERATE_CASE(STRING, VALUE) \
    case VALUE:                             \
        return #STRING;                     \
        break;
/// @endcond

namespace trolly {

/**
 * @brief Internal error typedef
 * 
 */
enum err_t : uint16_t { TROLLY_FOREACH_ERR(TROLLY_GENERATE_ENUM) };

/**
 * @brief Returns the string representation of the enum error code.
 * 
 * @param err_code Error code enum
 * @return String of the error
 */
std::string err_to_string(err_t err_code);

}  // namespace trolly

#endif  //TROLLY_ERRNO_H
