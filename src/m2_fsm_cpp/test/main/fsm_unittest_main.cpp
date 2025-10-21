#include "m2_fsm_cpp/test/fsm_unittest.h"
#include "trolly/log/trolly_logger_printf.h"

int main(int argc, char** argv)
{
    trolly::log::use_printf_logger();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
