#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_fsm_cpp/base/fsm_command_instance_id_allocator.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_fsm_cpp/base/fsm_subsystem.h"
#include "m2_fsm_cpp/base/fsm_subsystem_instance_id_allocator.h"
#include "m2_fsm_cpp/test/fsm_unittest.h"
namespace m2::fsm {

namespace {

class test_subsystem1 : public subsystem {
public:
    explicit test_subsystem1(const std::string& name)
        : subsystem(name){};
    void tick() noexcept override {}
};

class test_subsystem2 : public subsystem {
public:
    explicit test_subsystem2(const std::string& name)
        : subsystem(name)
    {}
    void tick() noexcept override {}
};

class test_command1 : public command {
public:
    explicit test_command1(const std::string& name)
        : command(name)
    {}
    void initialize() noexcept override {}
    void execute() noexcept override {}
    void end(bool /* interrupted */) noexcept override {}
    bool is_finished() noexcept override { return true; }
};

class test_command2 : public command {
public:
    explicit test_command2(const std::string& name)
        : command(name)
    {}
    void initialize() noexcept override {}
    void execute() noexcept override {}
    void end(bool /* interrupted */) noexcept override {}
    bool is_finished() noexcept override { return true; }
};

}  // namespace

TROLLY_TEST(base_test, subsystem_id_allocate)
{
    subsystem_instance_id_allocator id_allocator;
    auto last_id = id_allocator.allocate_id();
    TROLLY_EXPECT_EQ(id_allocator.allocate_id(), last_id + 1);
    TROLLY_EXPECT_EQ(id_allocator.allocate_id(), last_id + 2);
    TROLLY_EXPECT_EQ(id_allocator.allocate_id(), last_id + 3);
}

TROLLY_TEST(base_test, command_id_allocate)
{
    command_instance_id_allocator id_allocator;
    auto last_id = id_allocator.allocate_id();
    TROLLY_EXPECT_EQ(id_allocator.allocate_id(), last_id + 1);
    TROLLY_EXPECT_EQ(id_allocator.allocate_id(), last_id + 2);
    TROLLY_EXPECT_EQ(id_allocator.allocate_id(), last_id + 3);
}

TROLLY_TEST(base_test, command_add_requirements)
{
    auto& cs = command_scheduler::instance();
    auto& cmd_id_alloc = cs.command_id_allocator();

    test_subsystem1 ss1("a");
    ss1.set_instance_id(cmd_id_alloc.allocate_id());
    test_subsystem1 ss2("b");
    ss2.set_instance_id(cmd_id_alloc.allocate_id());
    test_subsystem2 ss3("c");
    ss3.set_instance_id(cmd_id_alloc.allocate_id());
    test_subsystem2 ss4("d");
    ss4.set_instance_id(cmd_id_alloc.allocate_id());

    test_command1 cmd("cmd");
    cmd.add_requirements(ss1, ss2);

    TROLLY_EXPECT_TRUE(cmd.has_requirement(ss1));
    TROLLY_EXPECT_TRUE(cmd.has_requirement(ss2));
    TROLLY_EXPECT_FALSE(cmd.has_requirement(ss3));
    TROLLY_EXPECT_FALSE(cmd.has_requirement(ss4));

    cmd.add_requirements(ss3, ss4);
    TROLLY_EXPECT_TRUE(cmd.has_requirement(ss1));
    TROLLY_EXPECT_TRUE(cmd.has_requirement(ss2));
    TROLLY_EXPECT_TRUE(cmd.has_requirement(ss3));
    TROLLY_EXPECT_TRUE(cmd.has_requirement(ss4));
}

TROLLY_TEST(base_test, register_unregister_subsystem)
{
    auto& cs = command_scheduler::instance();
    auto id = cs.register_subsystem(std::make_unique<test_subsystem1>("a"));
    TROLLY_EXPECT_NE(id, SUBSYSTEM_INSTANCE_ID_INVALID);
    TROLLY_EXPECT_NE(cs.get_subsystem(id), nullptr);
}

}  // namespace m2::fsm
