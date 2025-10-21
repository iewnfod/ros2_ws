
#include "m2_fsm_cpp/commands/fsm_wait_until_command.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_fsm_cpp/test/fsm_unittest.h"

namespace m2::fsm {

TROLLY_TEST(wait_until_command_test, condition_test)
{
    auto& cs = command_scheduler::instance();

    auto value = false;
    auto cmd_id =
        cs.schedule_command(std::make_unique<wait_until_command>([&value] { return value; }));
    TROLLY_EXPECT_NE(cmd_id, COMMAND_INSTANCE_ID_INVALID);
    TROLLY_EXPECT_NE(cs.get_command(cmd_id), nullptr);

    // tick once to schedule
    cs.tick();
    TROLLY_EXPECT_TRUE(cs.is_scheduled(cmd_id));

    // it should be still here since value is false
    cs.tick();
    TROLLY_EXPECT_NE(cs.get_command(cmd_id), nullptr);

    // it should be gone now
    value = true;
    cs.tick();
    TROLLY_EXPECT_EQ(cs.get_command(cmd_id), nullptr);
}

}  // namespace m2::fsm
