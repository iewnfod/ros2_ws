#ifndef TROLLY_ANY_PTR_H
#define TROLLY_ANY_PTR_H

#include "trolly/trolly_macro.h"

#ifdef TROLLY_DEBUG_MODE
#include <any>
#endif

namespace trolly {

class any_ptr {
public:
    any_ptr() noexcept = default;

    ~any_ptr() noexcept = default;

    template<typename T>
    any_ptr(T* p) noexcept  // NOLINT
#ifdef TROLLY_DEBUG_MODE
        : ptr_(p)
#else
        : ptr_(static_cast<void*>(p))
#endif
    {}

    template<typename T>
    TROLLY_NO_DISCARD T* cast() const noexcept
    {
#ifdef TROLLY_DEBUG_MODE
        return std::any_cast<T*>(ptr_);
#else
        return static_cast<T*>(ptr_);
#endif
    }

    template<typename T>
    any_ptr& operator=(T* p) noexcept
    {
#ifdef TROLLY_DEBUG_MODE
        ptr_ = p;
#else
        ptr_ = static_cast<void*>(p);
#endif
        return *this;
    }

private:
#ifdef TROLLY_DEBUG_MODE
    std::any ptr_;
#else
    void* ptr_ = nullptr;
#endif
    TROLLY_DEFAULT_COPY_AND_MOVE(any_ptr);
};

class const_any_ptr {
public:
    const_any_ptr() noexcept = default;

    ~const_any_ptr() noexcept = default;

    template<typename T>
    const_any_ptr(const T* p) noexcept  // NOLINT
#ifdef TROLLY_DEBUG_MODE
        : ptr_(p)
#else
        : ptr_(static_cast<const void*>(p))
#endif
    {}

    template<typename T>
    TROLLY_NO_DISCARD const T* cast() const noexcept
    {
#ifdef TROLLY_DEBUG_MODE
        return std::any_cast<const T*>(ptr_);
#else
        return static_cast<const T*>(ptr_);
#endif
    }

    template<typename T>
    const_any_ptr& operator=(const T* p) noexcept
    {
#ifdef TROLLY_DEBUG_MODE
        ptr_ = p;
#else
        ptr_ = static_cast<const void*>(p);
#endif
        return *this;
    }

private:
#ifdef TROLLY_DEBUG_MODE
    std::any ptr_;
#else
    const void* ptr_ = nullptr;
#endif
    TROLLY_DEFAULT_COPY_AND_MOVE(const_any_ptr);
};

#define TROLLY_ANY_PTR_CAST(ptr, type) ptr.cast<type>()

}  // namespace trolly

#endif
