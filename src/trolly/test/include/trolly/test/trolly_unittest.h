/**
 * @file trolly_unittest.h
 * @author Fernando (ferrrnnnandochen@gmail.com)
 * @brief Trolly unittest interface
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023 HKU Robocon Team
 *
 */

#ifndef TROLLY_UNITTEST_H
#define TROLLY_UNITTEST_H

#include "trolly/test/trolly_test.h"

class trolly_unittest : public ::testing::Test {
public:
    trolly_unittest() noexcept = default;

    ~trolly_unittest() noexcept override = default;

    void TestBody() override {}

    TROLLY_DISALLOW_COPY_AND_MOVE(trolly_unittest);
};

#endif
