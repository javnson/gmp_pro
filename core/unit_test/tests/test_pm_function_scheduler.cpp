/**
 * @file test_pm_function_scheduler.cpp
 * @brief Visual Studio native tests for the cooperative function scheduler.
 */

#include "vs_test_platform.h"
#include "vs_test_support.h"

#include <core/pm/function_scheduler/function_scheduler.h>

#include <cstring>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_test
{
namespace
{
struct task_data_t
{
    int calls;
    int busy_calls_remaining;
};

gmp_task_status_t test_task_handler(gmp_task_t* task)
{
    task_data_t* data = static_cast<task_data_t*>(task->user_data);
    ++data->calls;
    if (data->busy_calls_remaining > 0)
    {
        --data->busy_calls_remaining;
        return GMP_TASK_BUSY;
    }
    return GMP_TASK_DONE;
}
} // namespace

TEST_CLASS(FunctionSchedulerTests)
{
  public:
    TEST_METHOD_INITIALIZE(ResetHostTick)
    {
        gmp_test_system_tick = 0;
    }

    TEST_METHOD(PeriodicAndDisabledTasksRespectTheirTimingState)
    {
        gmp_scheduler_t scheduler{};
        task_data_t data{0, 0};
        gmp_task_t task{"periodic", test_task_handler, 10, 0, 1, &data, 99};

        gmp_scheduler_init(&scheduler);
        Assert::AreEqual<fast_gt>(0, gmp_scheduler_add_task(&scheduler, &task));
        Assert::AreEqual<fast_gt>(0, task.run_state);

        gmp_test_system_tick = 9;
        gmp_scheduler_dispatch(&scheduler);
        Assert::AreEqual(0, data.calls);

        gmp_test_system_tick = 10;
        gmp_scheduler_dispatch(&scheduler);
        Assert::AreEqual(1, data.calls);
        Assert::AreEqual<time_gt>(10, task.last_run);

        task.is_enabled = 0;
        gmp_test_system_tick = 30;
        gmp_scheduler_dispatch(&scheduler);
        Assert::AreEqual(1, data.calls);
    }

    TEST_METHOD(BusyTaskRetainsPriorityUntilItCompletes)
    {
        gmp_scheduler_t scheduler{};
        task_data_t busy_data{0, 2};
        task_data_t other_data{0, 0};
        gmp_task_t busy{"busy", test_task_handler, 0, 0, 1, &busy_data, 0};
        gmp_task_t other{"other", test_task_handler, 0, 0, 1, &other_data, 0};

        gmp_scheduler_init(&scheduler);
        Assert::AreEqual<fast_gt>(0, gmp_scheduler_add_task(&scheduler, &busy));
        Assert::AreEqual<fast_gt>(0, gmp_scheduler_add_task(&scheduler, &other));

        gmp_test_system_tick = 100;
        gmp_scheduler_dispatch(&scheduler);
        Assert::IsTrue(scheduler.blocking_task == &busy);

        gmp_test_system_tick = 101;
        gmp_scheduler_dispatch(&scheduler);
        Assert::IsTrue(scheduler.blocking_task == &busy);
        Assert::AreEqual(0, other_data.calls);

        gmp_test_system_tick = 102;
        gmp_scheduler_dispatch(&scheduler);
        Assert::IsNull(scheduler.blocking_task);
        Assert::AreEqual(3, busy_data.calls);
        Assert::AreEqual<time_gt>(102, busy.last_run);
        Assert::AreEqual<uint32_t>(2, scheduler.busy_cnt);
    }

    TEST_METHOD(TaskCapacityRejectsAdditionalRegistrations)
    {
        gmp_scheduler_t scheduler{};
        gmp_task_t tasks[GMP_SCHEDULER_MAX_TASKS + 1]{};

        gmp_scheduler_init(&scheduler);
        for (int index = 0; index < GMP_SCHEDULER_MAX_TASKS; ++index)
        {
            Assert::AreEqual<fast_gt>(0, gmp_scheduler_add_task(&scheduler, &tasks[index]));
        }
        Assert::AreNotEqual<fast_gt>(0, gmp_scheduler_add_task(&scheduler, &tasks[GMP_SCHEDULER_MAX_TASKS]));
        Assert::AreEqual<uint16_t>(GMP_SCHEDULER_MAX_TASKS, scheduler.task_count);
    }
};
} // namespace gmp_core_unit_test

