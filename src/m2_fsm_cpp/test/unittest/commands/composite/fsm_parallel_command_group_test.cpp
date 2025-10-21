
#include "m2_fsm_cpp/commands/composite/fsm_parallel_command_group.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_fsm_cpp/commands/fsm_functional_command.h"
#include "m2_fsm_cpp/test/fsm_unittest.h"

namespace m2::fsm {

namespace {

command_ptr make_command(
    bool& ran_initialize, bool& ran_execute, bool& ran_end, bool& ran_is_finished)
{
    return std::make_unique<functional_command>([&ran_initialize]() { ran_initialize = true; },
        [&ran_execute]() { ran_execute = true; },
        [&ran_end](bool /*interrupted */) { ran_end = true; },
        [&ran_is_finished]() {
            ran_is_finished = true;
            return true;
        });
}

}  // namespace

TROLLY_TEST(parallel_command_group_test, multiple_commands_test)
{
    auto& cs = command_scheduler::instance();

    bool cmd1_init = false;
    bool cmd1_exec = false;
    bool cmd1_end = false;
    bool cmd1_is_finished = false;

    bool cmd2_init = false;
    bool cmd2_exec = false;
    bool cmd2_end = false;
    bool cmd2_is_finished = false;

    auto cmd1 = make_command(cmd1_init, cmd1_exec, cmd1_end, cmd1_is_finished);
    auto cmd1_id = cs.command_id_allocator().allocate_id();
    cmd1->set_instance_id(cmd1_id);
    auto cmd2 = make_command(cmd2_init, cmd2_exec, cmd2_end, cmd2_is_finished);
    auto cmd2_id = cs.command_id_allocator().allocate_id();
    cmd2->set_instance_id(cmd2_id);

    auto scg_id = cs.schedule_command(
        std::make_unique<parallel_command_group>(std::move(cmd1), std::move(cmd2)));
    TROLLY_EXPECT_NE(scg_id, COMMAND_INSTANCE_ID_INVALID);
    auto* scg = dynamic_cast<parallel_command_group*>(cs.get_command(scg_id));
    TROLLY_EXPECT_NE(scg, nullptr);
    TROLLY_EXPECT_NE(scg->get_command(cmd1_id), nullptr);
    TROLLY_EXPECT_NE(scg->get_command(cmd2_id), nullptr);

    // tick once to schedule
    cs.tick();
    TROLLY_EXPECT_TRUE(cs.is_scheduled(scg_id));
    TROLLY_EXPECT_TRUE(cmd1_init);
    TROLLY_EXPECT_TRUE(cmd2_init);

    TROLLY_EXPECT_FALSE(cmd1_exec);
    TROLLY_EXPECT_FALSE(cmd1_end);
    TROLLY_EXPECT_FALSE(cmd1_is_finished);
    TROLLY_EXPECT_FALSE(cmd2_exec);
    TROLLY_EXPECT_FALSE(cmd2_end);
    TROLLY_EXPECT_FALSE(cmd2_is_finished);

    // both commands being ran and finished
    cs.tick();
    TROLLY_EXPECT_TRUE(cmd1_exec);
    TROLLY_EXPECT_TRUE(cmd1_end);
    TROLLY_EXPECT_TRUE(cmd1_is_finished);
    TROLLY_EXPECT_TRUE(cmd2_exec);
    TROLLY_EXPECT_TRUE(cmd2_end);
    TROLLY_EXPECT_TRUE(cmd2_is_finished);
}

}  // namespace m2::fsm
