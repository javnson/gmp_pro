/**
 * @file test_pm_workflow.cpp
 * @brief Visual Studio native tests for inline legacy workflow nodes.
 */

#include "vs_test_support.h"

#include <gmp_type.h>

static time_gt workflow_test_tick(void);
#define SPECIFY_WORKFLOW_TIMER workflow_test_tick
#include <core/pm/workflow/workflow.hpp>
#undef SPECIFY_WORKFLOW_TIMER

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

gmp_wf_node_base_gt* wf_end_node = nullptr;

namespace gmp_core_unit_test
{
namespace
{
time_gt workflow_test_tick_value = 0;

class exposed_node_t : public gmp_wf_node_gt
{
  public:
    using gmp_wf_node_gt::gmp_wf_node_gt;
    using gmp_wf_node_gt::node_routine;
    using gmp_wf_node_gt::node_transfer;
};

gmp_wf_node_base_gt* return_end(gmp_workflow_t*)
{
    return wf_end_node;
}
} // namespace

TEST_CLASS(WorkflowNodeTests)
{
  public:
    TEST_METHOD_INITIALIZE(ResetWorkflowTick)
    {
        workflow_test_tick_value = 0;
    }

    TEST_METHOD(NodeIdentifiersCanBeReadAndChanged)
    {
        gmp_wf_node_base_gt node(7);
        Assert::AreEqual<size_gt>(7, node.get_id());
        node.set_id(8);
        Assert::AreEqual<size_gt>(8, node.get_id());
    }

    TEST_METHOD(GeneralNodeUsesDefaultAndConfiguredCallbacks)
    {
        gmp_wf_node_base_gt end_node(gmp_workflow_t::gmp_wf_end_id);
        exposed_node_t node(7);
        wf_end_node = &end_node;

        Assert::IsTrue(node.node_routine(nullptr) == wf_end_node);
        Assert::IsNull(node.node_transfer(nullptr));

        node.set_routine_proc(return_end);
        node.set_transfer_proc(return_end);
        Assert::IsTrue(node.node_routine(nullptr) == wf_end_node);
        Assert::IsTrue(node.node_transfer(nullptr) == wf_end_node);
    }
};
} // namespace gmp_core_unit_test

static time_gt workflow_test_tick(void)
{
    return gmp_core_unit_test::workflow_test_tick_value;
}

