#ifndef M2_SIC_ROBOT_MODES_MANUAL_MODE_H
#define M2_SIC_ROBOT_MODES_MANUAL_MODE_H

#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_mode.h"

namespace m2::sic_robot {

class manual_mode : public fsm::input::ps_mode {
public:
    explicit manual_mode() noexcept;
    ~manual_mode() noexcept override = default;

    void tick() noexcept override;
    void on_enter() noexcept override;
    void on_exit() noexcept override;
    void on_input(const ps_input_data_type& data) noexcept override;

private:
    TROLLY_DISALLOW_COPY_AND_MOVE(manual_mode);
};

}  // namespace m2::sic_robot

#endif  // M2_SIC_ROBOT_MODES_MANUAL_MODE_H
