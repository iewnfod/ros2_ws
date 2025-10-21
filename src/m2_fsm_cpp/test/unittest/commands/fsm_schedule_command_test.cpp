
#include "m2_fsm_cpp/commands/fsm_schedule_command.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_fsm_cpp/test/fsm_unittest.h"

namespace m2::fsm {

namespace {

class test_command : public command {
public:
    explicit test_command(const std::string& name)
        : command(name)
    {}
    void initialize() noexcept override {}
    void execute() noexcept override {}
    void end(bool /* interrupted */) noexcept override {}
    bool is_finished() noexcept override { return true; }
};

}  // namespace

TROLLY_TEST(schedule_command_test, multiple_schedule_test)
{
    auto& cs = command_scheduler::instance();

    auto cmd1 = std::make_unique<test_command>("a");
    auto cmd1_id = cs.command_id_allocator().allocate_id();
    cmd1->set_instance_id(cmd1_id);
    auto cmd2 = std::make_unique<test_command>("b");
    auto cmd2_id = cs.command_id_allocator().allocate_id();
    cmd2->set_instance_id(cmd2_id);

    auto sc_id =
        cs.schedule_command(std::make_unique<schedule_command>(std::move(cmd1), std::move(cmd2)));
    TROLLY_EXPECT_NE(sc_id, COMMAND_INSTANCE_ID_INVALID);
    TROLLY_EXPECT_NE(cs.get_command(sc_id), nullptr);

    // tick once to schedule
    cs.tick();
    TROLLY_EXPECT_TRUE(cs.is_scheduled(sc_id));

    // let schedule command run and schedule the two cmds
    cs.tick();
    // schedule command should be gone
    TROLLY_EXPECT_EQ(cs.get_command(sc_id), nullptr);
    TROLLY_EXPECT_TRUE(cs.is_scheduled(cmd1_id));
    TROLLY_EXPECT_TRUE(cs.is_scheduled(cmd2_id));

    cs.tick();
    TROLLY_EXPECT_EQ(cs.get_command(cmd1_id), nullptr);
    TROLLY_EXPECT_EQ(cs.get_command(cmd2_id), nullptr);
}

}  // namespace m2::fsm
