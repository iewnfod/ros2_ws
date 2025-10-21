/**
 * @file trolly_macro.h
 * @author Anthony Law (anthlaw@connect.hku.hk)
 * @brief Macros
 * @version 0.1
 * @date 2023-10-25
 * 
 * @copyright Copyright (c) 2023 HKU Robocon Team
 * 
 */

#ifndef TROLLY_MACRO_H
#define TROLLY_MACRO_H

#include "trolly/portability/trolly_portability.h"

#include <cassert>

/**
 * @brief Indicates the function/variable is not in use within the code
 * 
 */
#define TROLLY_UNUSED(x) (void)(x)

#define TROLLY_NO_DISCARD [[nodiscard]]

#ifdef TROLLY_COMPILER_GNU
#define TROLLY_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define TROLLY_ALWAYS_INLINE inline
#endif

#define TROLLY_NO_DISCARD_INLINE TROLLY_NO_DISCARD TROLLY_ALWAYS_INLINE

#ifdef TROLLY_COMPILER_GNU
#define TROLLY_LIKELY(x) __builtin_expect((x), 1)
#else
#define TROLLY_LIKELY(x) x
#endif

#ifdef TROLLY_COMPILER_GNU
#define TROLLY_UNLIKELY(x) __builtin_expect((x), 0)
#else
#define TROLLY_LIKELY(x) x
#endif

#define TROLLY_ASSERT(...) assert(__VA_ARGS__)

#define TROLLY_EXPECTS(...) TROLLY_ASSERT(__VA_ARGS__)

#define TROLLY_ENSURES(...) TROLLY_ASSERT(__VA_ARGS__)

#ifdef TROLLY_COMPILER_GNU
#define TROLLY_PACKED_STRUCT(name) struct __attribute__((packed)) #name
#define TROLLY_ALIGNED_STRUCT(name, align) struct __attribute__((aligned(#align))) #name
#define TROLLY_PACKED_ALIGNED_STRUCT(name, align) \
    struct _attribute__((packed, aligned(#align))) #name
#else
#define TROLLY_PACKED_STRUCT(name) struct #name
#define TROLLY_ALIGNED_STRUCT(name) struct #name
#define TROLLY_PACKED_ALIGNED_STRUCT(name) struct #name
#endif

/**
 * @brief Disallows a class to be copied and assigned
 * 
 */
#define TROLLY_DISALLOW_COPY_AND_ASSIGN(TypeName) \
public:                                           \
    TypeName(const TypeName&) = delete;           \
    TypeName& operator=(const TypeName&) = delete

/**
 * @brief Disallows a class to be moved
 * 
 */
#define TROLLY_DISALLOW_MOVE(TypeName) \
public:                                \
    TypeName(TypeName&&) = delete;     \
    TypeName& operator=(TypeName&&) = delete

/**
 * @brief Disallows a class to be copied and moved
 * 
 */
#define TROLLY_DISALLOW_COPY_AND_MOVE(TypeName) \
    TROLLY_DISALLOW_MOVE(TypeName);             \
    TROLLY_DISALLOW_COPY_AND_ASSIGN(TypeName)

/**
 * @brief Defaults a class to be copied and assigned
 * 
 */
#define TROLLY_DEFAULT_COPY_AND_ASSIGN(TypeName) \
public:                                          \
    TypeName(const TypeName&) = default;         \
    TypeName& operator=(const TypeName&) = default

/**
 * @brief Defaults a class to be moved
 * 
 */
#define TROLLY_DEFAULT_MOVE(TypeName) \
public:                               \
    TypeName(TypeName&&) = default;   \
    TypeName& operator=(TypeName&&) = default

/**
 * @brief Defaults a class to be copied and moved
 * 
 */
#define TROLLY_DEFAULT_COPY_AND_MOVE(TypeName) \
    TROLLY_DEFAULT_MOVE(TypeName);             \
    TROLLY_DEFAULT_COPY_AND_ASSIGN(TypeName)

#endif  //TROLLY_MACRO_H
