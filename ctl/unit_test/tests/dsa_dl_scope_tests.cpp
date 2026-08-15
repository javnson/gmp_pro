/**
 * @file dsa_dl_scope_tests.cpp
 * @brief Hosted regression tests for the DSA-backed Data Link Scope adapter.
 */

#include "vs_test_support.h"

#include <gmp_type.h>
#include <ctl/math_block/gmp_math.h>
#include <core/dev/datalink/datalink.h>
#include <ctl/component/dsa/dsa_dl_scope.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

extern "C"
{
time_gt gmp_base_get_system_tick(void) { return 0U; }
void gmp_base_enter_critical(void) {}
void gmp_base_leave_critical(void) {}
}

namespace gmp_ctl_unit_test
{
TEST_CLASS(DsaDataLinkScopeTests)
{
  public:
    TEST_METHOD(WorkspaceOwnsMetadataAndRecorderDimensions)
    {
        gmp_datalink_t datalink{};
        ctl_dsa_dl_scope_t scope{};
        ctrl_gt workspace[CTL_DSA_DL_SCOPE_STORAGE_ELEMENTS(2U, 4U)]{};
        gmp_dev_dl_init(&datalink);

        Assert::AreEqual<fast_gt>(
            1, ctl_init_dsa_dl_scope_workspace(
                   &scope, &datalink, 0x60U, "Test Scope", workspace,
                   static_cast<uint32_t>(sizeof(workspace) / sizeof(workspace[0])),
                   2U, 1000U));
        Assert::AreEqual<uint16_t>(2U, scope.resource.channels);
        Assert::AreEqual<uint32_t>(4U, scope.resource.depth);
        Assert::AreEqual<uint32_t>(32U, scope.resource.byte_length);
        Assert::AreEqual<uint32_t>(8U, scope.recorder.mem.capacity);
        Assert::AreEqual<uint32_t>(4U, scope.recorder.depth);
        Assert::AreEqual<int>(static_cast<int>(DSA_TRIGGER_OPTION_RISING_EDGE),
                              static_cast<int>(scope.trigger_mode));
        Assert::AreEqual<double>(0.0, static_cast<double>(scope.trigger_level));
        Assert::IsTrue(scope.buffer == workspace);
        Assert::IsTrue(scope.history == workspace + 8U);
    }

    TEST_METHOD(TwoChannelCaptureKeepsChannelsIndependent)
    {
        gmp_datalink_t datalink{};
        ctl_dsa_dl_scope_t scope{};
        ctrl_gt workspace[CTL_DSA_DL_SCOPE_STORAGE_ELEMENTS(2U, 4U)]{};
        gmp_dev_dl_init(&datalink);
        Assert::AreEqual<fast_gt>(
            1, ctl_init_dsa_dl_scope_workspace(
                   &scope, &datalink, 0x60U, "Test Scope", workspace,
                   static_cast<uint32_t>(sizeof(workspace) / sizeof(workspace[0])),
                   2U, 1000U));

        ctl_step_dsa_dl_scope_2ch(&scope, real2ctrl(-2.0F), real2ctrl(20.0F));
        ctl_step_dsa_dl_scope_2ch(&scope, real2ctrl(-1.0F), real2ctrl(21.0F));
        ctl_step_dsa_dl_scope_2ch(&scope, real2ctrl(1.0F), real2ctrl(22.0F));
        ctl_step_dsa_dl_scope_2ch(&scope, real2ctrl(2.0F), real2ctrl(23.0F));

        Assert::AreEqual<int>(static_cast<int>(GMP_SCOPE_STATE_READY),
                              static_cast<int>(scope.state));
        Assert::AreEqual<uint32_t>(1U, static_cast<uint32_t>(scope.generation));
        const float expected_0[] = {-2.0F, -1.0F, 1.0F, 2.0F};
        const float expected_1[] = {20.0F, 21.0F, 22.0F, 23.0F};
        for (uint32_t index = 0U; index < 4U; ++index)
        {
            Assert::AreEqual<float>(expected_0[index], scope.buffer[index]);
            Assert::AreEqual<float>(expected_1[index], scope.buffer[4U + index]);
        }
    }

    TEST_METHOD(WorkspaceRejectsInvalidDimensions)
    {
        gmp_datalink_t datalink{};
        ctl_dsa_dl_scope_t scope{};
        ctrl_gt workspace[16]{};
        gmp_dev_dl_init(&datalink);
        Assert::AreEqual<fast_gt>(
            0, ctl_init_dsa_dl_scope_workspace(&scope, &datalink, 0x60U,
                                               "Bad", workspace, 15U, 2U, 1000U));
        Assert::AreEqual<fast_gt>(
            0, ctl_init_dsa_dl_scope_workspace(&scope, &datalink, 0x60U,
                                               "Bad", workspace, 16U, 5U, 1000U));
    }
};
} // namespace gmp_ctl_unit_test
