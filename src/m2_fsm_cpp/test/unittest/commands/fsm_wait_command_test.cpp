
#include "m2_fsm_cpp/commands/fsm_wait_command.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_fsm_cpp/test/fsm_unittest.h"

#include <thread>

namespace m2::fsm {

using namespace std::chrono_literals;

TROLLY_TEST(wait_command_test, wait_test)
{
    auto& cs = command_scheduler::instance();

    auto cmd_id = cs.schedule_command(std::make_unique<wait_command>(100));
    TROLLY_EXPECT_NE(cmd_id, COMMAND_INSTANCE_ID_INVALID);
    auto* cmd = cs.get_command(cmd_id);
    TROLLY_EXPECT_NE(cmd, nullptr);

    // tick once to schedule the command first
    cs.tick();
    TROLLY_EXPECT_TRUE(cs.is_scheduled(cmd_id));

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time)
               .count()
        <= 100) {
        std::this_thread::sleep_for(5ms);
    }
}

}  // namespace m2::fsm
