/**
 * @file test_pm_duff_fsm.cpp
 * @brief Visual Studio native tests for the Duff-style cooperative FSM.
 */

#include "vs_test_platform.h"
#include "vs_test_support.h"

#include <core/pm/duff_fsm/duff_fsm.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_test
{
namespace
{
struct test_fsm_t
{
    fsm_ctx_t context;
    int completed_steps;
};

fsm_ret_t run_test_fsm(test_fsm_t* state)
{
    fsm_ctx_t* context = &state->context;

    _FSM_START(context)
    _CASE_START(0)
    ++state->completed_steps;
    fsm_wait(context, 5);
    fsm_goto(context, 1);
    _CASE_END
    _CASE_START(1)
    ++state->completed_steps;
    _CASE_END
    _FSM_END(context)
}
} // namespace

TEST_CLASS(DuffFsmTests)
{
  public:
    TEST_METHOD_INITIALIZE(ResetHostTick)
    {
        gmp_test_system_tick = 0;
    }

    TEST_METHOD(WaitAndGotoResumeAcrossInvocations)
    {
        test_fsm_t state{{0, 0, 0}, 0};

        gmp_test_system_tick = 100;
        Assert::AreEqual<int>(FSM_RET_YIELD, run_test_fsm(&state));
        Assert::AreEqual<uint16_t>(FSM_MAGIC_WORD, state.context.magic);
        Assert::AreEqual(1, state.completed_steps);

        gmp_test_system_tick = 104;
        Assert::AreEqual<int>(FSM_RET_YIELD, run_test_fsm(&state));
        Assert::AreEqual(1, state.completed_steps);

        gmp_test_system_tick = 105;
        Assert::AreEqual<int>(FSM_RET_YIELD, run_test_fsm(&state));
        gmp_test_system_tick = 106;
        Assert::AreEqual<int>(FSM_RET_DONE, run_test_fsm(&state));
        Assert::AreEqual(2, state.completed_steps);
    }

    TEST_METHOD(ResetRestartsTheStateMachine)
    {
        test_fsm_t state{{FSM_MAGIC_WORD, 1, 0}, 0};
        fsm_reset(&state.context);
        Assert::AreEqual<uint16_t>(0, state.context.magic);

        gmp_test_system_tick = 200;
        Assert::AreEqual<int>(FSM_RET_YIELD, run_test_fsm(&state));
        Assert::AreEqual(1, state.completed_steps);
    }

    TEST_METHOD(UnknownStateReturnsErrorAndClearsContext)
    {
        test_fsm_t state{{FSM_MAGIC_WORD, 0xFFFFFFFFu, 0}, 0};
        Assert::AreEqual<int>(FSM_RET_ERROR, run_test_fsm(&state));
        Assert::AreEqual<uint16_t>(0, state.context.magic);
        Assert::AreEqual<uint32_t>(0, state.context.step);
    }
};
} // namespace gmp_core_unit_test

