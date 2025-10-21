#ifndef TROLLY_PORTABILITY_H
#define TROLLY_PORTABILITY_H

#if defined(__GNUC__) || defined(__clang__)
#define TROLLY_COMPILER_GNU
#endif

#if _MSC_VER
#define TROLLY_COMPILER_MSVC
#endif

#endif
