/**
 * @file fsm_unittest.h
 * @author Fernando (ferrrnnnandochen@gmail.com)
 * @brief fsm unittest interface
 * @version 0.1
 * @date 2024-02-25
 *
 * @copyright Copyright (c) 2024 HKU Robocon Team
 *
 */

#ifndef FSM_UNITTEST_H
#define FSM_UNITTEST_H

#include "m2_fsm_cpp/test/fsm_test.h"
#include "trolly/trolly_macro.h"

class fsm_unittest : public ::testing::Test {
public:
    fsm_unittest() noexcept = default;

    ~fsm_unittest() noexcept override = default;

    void TestBody() override {}

    TROLLY_DISALLOW_COPY_AND_MOVE(fsm_unittest);
};

#endif
