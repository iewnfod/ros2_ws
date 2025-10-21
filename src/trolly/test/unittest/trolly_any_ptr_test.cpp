#include "trolly/trolly_any_ptr.h"

#include "trolly/test/trolly_unittest.h"

namespace {

struct test_t {};

}  // namespace

TEST(any_test, any_ptr)  // NOLINT
{
    int a1 = 10;
    trolly::any_ptr ptr1(&a1);
    TROLLY_EXPECT_EQ(TROLLY_ANY_PTR_CAST(ptr1, int), &a1);

    test_t a2;
    trolly::any_ptr ptr2;
    ptr2 = &a2;
    TROLLY_EXPECT_EQ(TROLLY_ANY_PTR_CAST(ptr2, test_t), &a2);

    test_t a3;
    trolly::any_ptr ptr3 = &a3;
    TROLLY_EXPECT_EQ(TROLLY_ANY_PTR_CAST(ptr3, test_t), &a3);

    ptr3 = ptr2;
    TROLLY_EXPECT_EQ(TROLLY_ANY_PTR_CAST(ptr3, test_t), TROLLY_ANY_PTR_CAST(ptr2, test_t));
}

TEST(any_test, const_any_ptr)  // NOLINT
{
    // Notes: pointer stored inside const_any_ptr is not const, only the actual object pointed to is const

    int a1 = 10;                      // non-const object
    trolly::const_any_ptr ptr1(&a1);  // constructor can accept raw non-const ptr
    TROLLY_EXPECT_EQ(TROLLY_ANY_PTR_CAST(ptr1, int), &a1);

    const test_t a2;                   // const object
    trolly::const_any_ptr ptr2 = &a2;  // constructor can accept raw const ptr
    TROLLY_EXPECT_EQ(TROLLY_ANY_PTR_CAST(ptr2, test_t), &a2);

    const test_t a3;
    trolly::const_any_ptr ptr3;
    ptr3 = &a3;  // assignment from raw const ptr
    TROLLY_EXPECT_EQ(TROLLY_ANY_PTR_CAST(ptr3, test_t), &a3);

    ptr3 = ptr2;  // assignment from const_any_ptr
    TROLLY_EXPECT_EQ(TROLLY_ANY_PTR_CAST(ptr3, test_t), TROLLY_ANY_PTR_CAST(ptr2, test_t));
}
