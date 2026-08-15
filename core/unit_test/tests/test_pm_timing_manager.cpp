/**
 * @file test_pm_timing_manager.cpp
 * @brief Visual Studio native tests for the legacy timing manager.
 */

#include "vs_test_support.h"

#include <core/pm/timing/timing_manager.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_test
{
namespace
{
time_gt test_tick = 0;

time_gt get_test_tick(void)
{
    return test_tick;
}
} // namespace

TEST_CLASS(TimingManagerTests)
{
  public:
    TEST_METHOD_INITIALIZE(ResetTimer)
    {
        test_tick = 0;
    }

    TEST_METHOD(AutomaticCycleAccountingHandlesTickWrap)
    {
        timer_mgr_t manager{};
        time_tick_t moment{};
        time_tick_t current{};

        test_tick = 90;
        reg_timer_mgr(&manager, 100, get_test_tick, 1);
        get_current_time_tick(&manager, &moment);
        Assert::AreEqual<time_gt>(0, moment.cycle);
        Assert::AreEqual<time_gt>(90, moment.tick);

        test_tick = 99;
        Assert::AreEqual<time_gt>(9, get_delta_time(&manager, &moment));
        test_tick = 10;
        Assert::AreEqual<time_gt>(20, get_delta_time(&manager, &moment));
        get_current_time_tick(&manager, &current);
        Assert::AreEqual<time_gt>(1, current.cycle);
        Assert::AreEqual<time_gt>(10, current.tick);
    }

    TEST_METHOD(ManualCycleAndNullInputsAreHandled)
    {
        timer_mgr_t manager{};
        time_tick_t moment{};

        reg_timer_mgr(&manager, 100, get_test_tick, 0);
        step_cycle_cnt(&manager);
        Assert::AreEqual<time_gt>(1, manager.last_update_time.cycle);
        Assert::AreEqual<time_gt>(0, get_delta_time(nullptr, &moment));
        Assert::AreEqual<time_gt>(0, get_delta_time(&manager, nullptr));
    }
};
} // namespace gmp_core_unit_test

