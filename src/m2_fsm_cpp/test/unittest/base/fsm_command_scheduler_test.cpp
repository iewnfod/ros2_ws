#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_fsm_cpp/base/fsm_subsystem.h"
#include "m2_fsm_cpp/test/fsm_unittest.h"
#include "trolly/trolly_macro.h"
namespace m2::fsm {

namespace {

class test_subsystem : public subsystem {
public:
    explicit test_subsystem(const std::string& name)
        : subsystem(name){};
    void tick() noexcept override { tick_count_++; }
    TROLLY_NO_DISCARD_INLINE uint32_t tick_count() const { return tick_count_; }

private:
    uint32_t tick_count_{0};
};

class test_command : public command {
public:
    explicit test_command(const std::string& name)
        : command(name)
    {}
    void initialize() noexcept override { initialize_count_++; }
    void execute() noexcept override { execute_count_++; }
    void end(bool /* interrupted */) noexcept override { end_count_++; }
    bool is_finished() noexcept override
    {
        is_finished_count_++;
        return finished_val_;
    }
    TROLLY_NO_DISCARD_INLINE uint32_t initialize_count() const { return initialize_count_; }
    TROLLY_NO_DISCARD_INLINE uint32_t execute_count() const { return execute_count_; }
    TROLLY_NO_DISCARD_INLINE uint32_t end_count() const { return end_count_; }
    TROLLY_NO_DISCARD_INLINE uint32_t is_finished_count() const { return is_finished_count_; }
    void set_finished(bool val) { finished_val_ = val; }

private:
    bool finished_val_{false};
    uint32_t initialize_count_{0};
    uint32_t execute_count_{0};
    uint32_t end_count_{0};
    uint32_t is_finished_count_{0};
};

}  // namespace

TROLLY_TEST(command_scheduler_test, register_unregister_subsystem)
{
    auto& cs = command_scheduler::instance();
    auto id = cs.register_subsystem(std::make_unique<test_subsystem>("a"));
    TROLLY_EXPECT_NE(id, SUBSYSTEM_INSTANCE_ID_INVALID);
    TROLLY_EXPECT_NE(cs.get_subsystem(id), nullptr);
}

TROLLY_TEST(command_scheduler_test, simple_tick_test)
{
    auto& cs = command_scheduler::instance();

    auto ss_id = cs.register_subsystem(std::make_unique<test_subsystem>("a"));
    TROLLY_EXPECT_NE(ss_id, SUBSYSTEM_INSTANCE_ID_INVALID);
    auto* ss = dynamic_cast<test_subsystem*>(cs.get_subsystem(ss_id));
    TROLLY_EXPECT_NE(ss, nullptr);

    auto cmd_id = cs.schedule_command(std::make_unique<test_command>("b"));
    TROLLY_EXPECT_NE(cmd_id, COMMAND_INSTANCE_ID_INVALID);
    auto* cmd = dynamic_cast<test_command*>(cs.get_command(cmd_id));
    TROLLY_EXPECT_NE(cmd, nullptr);
    cmd->set_finished(false);

    cs.tick();
    TROLLY_EXPECT_EQ(ss->tick_count(), 1);
    TROLLY_EXPECT_EQ(cmd->initialize_count(), 1);
    TROLLY_EXPECT_TRUE(cs.is_scheduled(cmd_id));
    cs.tick();
    TROLLY_EXPECT_EQ(ss->tick_count(), 2);
    TROLLY_EXPECT_EQ(cmd->execute_count(), 1);
    TROLLY_EXPECT_EQ(cmd->is_finished_count(), 1);
    TROLLY_EXPECT_EQ(cmd->end_count(), 0);

    // erase the command by cancelling it and tick it away
    cmd->set_finished(true);
    cs.cancel_command(cmd_id);
    cs.tick();
    TROLLY_EXPECT_EQ(ss->tick_count(), 3);
    TROLLY_EXPECT_EQ(cs.get_command(cmd_id), nullptr);
}

TROLLY_TEST(command_scheduler_test, cancel_command_test)
{
    auto& cs = command_scheduler::instance();

    auto cmd_id = cs.schedule_command(std::make_unique<test_command>("a"));
    TROLLY_EXPECT_NE(cmd_id, COMMAND_INSTANCE_ID_INVALID);
    auto* cmd = dynamic_cast<test_command*>(cs.get_command(cmd_id));
    TROLLY_EXPECT_NE(cmd, nullptr);
    cmd->set_finished(false);

    // tick once schedule the command
    cs.tick();
    TROLLY_EXPECT_EQ(cmd->initialize_count(), 1);
    TROLLY_EXPECT_TRUE(cs.is_scheduled(cmd_id));

    // tick once to execute it, it should be not be ended
    cs.tick();
    TROLLY_EXPECT_EQ(cmd->execute_count(), 1);
    TROLLY_EXPECT_EQ(cmd->is_finished_count(), 1);
    TROLLY_EXPECT_EQ(cmd->end_count(), 0);

    // cancel the command
    cs.cancel_command(cmd_id);

    // tick once to cancel it, it should be ended now
    cs.tick();
    TROLLY_EXPECT_EQ(cs.get_command(cmd_id), nullptr);
}

TROLLY_TEST(command_scheduler_test, cancel_all_commands_test)
{
    auto& cs = command_scheduler::instance();

    auto cmd1_id = cs.schedule_command(std::make_unique<test_command>("a"));
    TROLLY_EXPECT_NE(cmd1_id, COMMAND_INSTANCE_ID_INVALID);
    auto* cmd1 = dynamic_cast<test_command*>(cs.get_command(cmd1_id));
    TROLLY_EXPECT_NE(cmd1, nullptr);
    cmd1->set_finished(false);

    auto cmd2_id = cs.schedule_command(std::make_unique<test_command>("b"));
    TROLLY_EXPECT_NE(cmd2_id, COMMAND_INSTANCE_ID_INVALID);
    auto* cmd2 = dynamic_cast<test_command*>(cs.get_command(cmd2_id));
    TROLLY_EXPECT_NE(cmd2, nullptr);
    cmd2->set_finished(false);

    // tick once schedule the command
    cs.tick();
    TROLLY_EXPECT_EQ(cmd1->initialize_count(), 1);
    TROLLY_EXPECT_TRUE(cs.is_scheduled(cmd1_id));
    TROLLY_EXPECT_EQ(cmd2->initialize_count(), 1);
    TROLLY_EXPECT_TRUE(cs.is_scheduled(cmd2_id));

    // tick once to execute it, it should be not be ended
    cs.tick();
    TROLLY_EXPECT_EQ(cmd1->execute_count(), 1);
    TROLLY_EXPECT_EQ(cmd1->is_finished_count(), 1);
    TROLLY_EXPECT_EQ(cmd1->end_count(), 0);
    TROLLY_EXPECT_EQ(cmd2->execute_count(), 1);
    TROLLY_EXPECT_EQ(cmd2->is_finished_count(), 1);
    TROLLY_EXPECT_EQ(cmd2->end_count(), 0);

    // cancel all commands
    cs.cancel_all_commands();

    // tick once to cancel it, it should be ended now
    cs.tick();
    TROLLY_EXPECT_EQ(cs.get_command(cmd1_id), nullptr);
    TROLLY_EXPECT_EQ(cs.get_command(cmd2_id), nullptr);
}

}  // namespace m2::fsm
