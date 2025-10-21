
#include "m2_fsm_cpp/commands/fsm_functional_command.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_fsm_cpp/test/fsm_unittest.h"

namespace m2::fsm {

TROLLY_TEST(functional_command_test, multiple_schedule_test)
{
    auto& cs = command_scheduler::instance();

    bool ran_initialize = false;
    bool ran_execute = false;
    bool ran_end = false;
    bool ran_is_finished = false;
    auto cmd = std::make_unique<functional_command>([&ran_initialize]() { ran_initialize = true; },
        [&ran_execute]() { ran_execute = true; },
        [&ran_end](bool /*interrupted */) { ran_end = true; },
        [&ran_is_finished]() {
            ran_is_finished = true;
            return true;
        });
    auto id = cs.schedule_command(std::move(cmd));
    TROLLY_EXPECT_NE(id, COMMAND_INSTANCE_ID_INVALID);

    // tick once to schedule
    cs.tick();
    TROLLY_EXPECT_TRUE(cs.is_scheduled(id));
    TROLLY_EXPECT_TRUE(ran_initialize);
    TROLLY_EXPECT_FALSE(ran_execute);
    TROLLY_EXPECT_FALSE(ran_end);
    TROLLY_EXPECT_FALSE(ran_is_finished);

    cs.tick();
    TROLLY_EXPECT_TRUE(ran_execute);
    TROLLY_EXPECT_TRUE(ran_end);
    TROLLY_EXPECT_TRUE(ran_is_finished);
}

}  // namespace m2::fsm
