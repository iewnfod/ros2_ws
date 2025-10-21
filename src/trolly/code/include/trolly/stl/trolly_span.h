#ifndef TROLLY_SPAN_H
#define TROLLY_SPAN_H

#include <span>

namespace trolly {

template<typename T>
using span = std::span<T>;

}  // namespace trolly

#endif
