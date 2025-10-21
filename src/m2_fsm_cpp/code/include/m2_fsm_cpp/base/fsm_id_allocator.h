#ifndef M2_FSM_CPP_BASE_FSM_ID_ALLOCATOR_H
#define M2_FSM_CPP_BASE_FSM_ID_ALLOCATOR_H

#include "trolly/trolly_macro.h"

#include <mutex>

namespace m2::fsm {

template<typename ID_TYPE>
class id_allocator {
public:
    explicit id_allocator() noexcept = default;
    virtual ~id_allocator() noexcept = default;

    ID_TYPE allocate_id() noexcept
    {
        const std::scoped_lock lock(id_count_mtx_);
        return id_count_++;
    }

private:
    static inline std::mutex id_count_mtx_;
    static inline ID_TYPE id_count_{0};

    TROLLY_DISALLOW_COPY_AND_MOVE(id_allocator);
};

}  // namespace m2::fsm

#endif  // M2_FSM_CPP_BASE_FSM_ID_ALLOCATOR_H
