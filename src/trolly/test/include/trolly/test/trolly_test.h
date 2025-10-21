/**
 * @file trolly_test.h
 * @author Fernando (ferrrnnnandochen@gmail.com)
 * @brief Trolly test defines and macros
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023 HKU Robocon Team
 *
 */

#ifndef TROLLY_TEST_H
#define TROLLY_TEST_H

#include "trolly/trolly_macro.h"

#include <gtest/gtest.h>

// clang-format off
#define TROLLY_EXPECT_TRUE(...) EXPECT_TRUE(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_FALSE(...) EXPECT_FALSE(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_ASSERT_TRUE(...) ASSERT_TRUE(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_ASSERT_FALSE(...) ASSERT_FALSE(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_EQ(...) EXPECT_EQ(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_NE(...) EXPECT_NE(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_LE(...) EXPECT_LE(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_LT(...) EXPECT_LT(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_GE(...) EXPECT_GE(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_GT(...) EXPECT_GT(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_NEAR(...) EXPECT_NEAR(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_DOUBLE_EQ(...) EXPECT_DOUBLE_EQ(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_STREQ(...) EXPECT_STREQ(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
#define TROLLY_EXPECT_STRNE(...) EXPECT_STRNE(__VA_ARGS__) /* NOLINT(hicpp-vararg, bugprone-branch-clone) */
// clang-format on

#define TROLLY_TEST(...) TEST(__VA_ARGS__)      // NOLINT
#define TROLLY_TEST_F(...) TEST_F(__VA_ARGS__)  // NOLINT

#endif
