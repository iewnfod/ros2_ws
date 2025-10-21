#include "trolly/math/trolly_math_util.h"
#include "trolly/test/trolly_unittest.h"

TROLLY_TEST(math_test, is_close)
{
    double num1 = 0.0;
    double num2 = 0.001;
    double num3 = 0.000001;
    TROLLY_EXPECT_FALSE(trolly::math::is_close(num1, num2));
    TROLLY_EXPECT_TRUE(trolly::math::is_close(num1, num3));
    TROLLY_EXPECT_TRUE(trolly::math::is_close(num1, num2, 1e-3));
}
