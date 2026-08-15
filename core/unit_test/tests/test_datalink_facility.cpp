/**
 * @file test_datalink_facility.cpp
 * @brief Native tests for intrusive Data Link facility registration.
 */

#include "vs_test_support.h"

extern "C"
{
#include <core/dev/datalink/datalink.h>
}

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_test
{
namespace
{
struct fake_facility_t
{
    gmp_dl_facility_t facility;
    uint32_t calls;
};

fast_gt dispatch_fake(gmp_dl_facility_t* facility, gmp_datalink_t* datalink)
{
    fake_facility_t* owner = static_cast<fake_facility_t*>(facility->owner);
    owner->calls++;
    gmp_dev_dl_reply_ack_null(datalink);
    return 1;
}

void init_fake(fake_facility_t& object, gmp_dl_facility_type_t type,
               uint16_t base, uint16_t count)
{
    object.calls = 0U;
    gmp_dl_facility_init(&object.facility, type, base, count,
                         dispatch_fake, &object);
}
} // namespace

TEST_CLASS(DataLinkFacilityTests)
{
  public:
    TEST_METHOD(FacilityHeaderStartsWithListThenType)
    {
        Assert::AreEqual<size_t>(0U, offsetof(gmp_dl_facility_t, list));
        Assert::AreEqual<size_t>(sizeof(gmp_list),
                                 offsetof(gmp_dl_facility_t, type));
    }

    TEST_METHOD(AppendPreservesOrderAndRejectsOverlap)
    {
        gmp_datalink_t datalink{};
        fake_facility_t tunable{};
        fake_facility_t memory{};
        fake_facility_t overlap{};
        gmp_dev_dl_init(&datalink);
        init_fake(tunable, GMP_DL_FACILITY_TUNABLE, 0x30U, 2U);
        init_fake(memory, GMP_DL_FACILITY_MEMORY, 0x50U, 2U);
        init_fake(overlap, GMP_DL_FACILITY_USER, 0x31U, 2U);

        Assert::AreEqual<fast_gt>(
            1, gmp_dev_dl_append_facility(&datalink, &tunable.facility));
        Assert::AreEqual<fast_gt>(
            1, gmp_dev_dl_append_facility(&datalink, &memory.facility));
        Assert::AreEqual<fast_gt>(
            0, gmp_dev_dl_append_facility(&datalink, &overlap.facility));
        Assert::AreEqual<uint16_t>(2U, datalink.facility_count);
        Assert::IsTrue(datalink.facility_list.next == &tunable.facility.list);
        Assert::IsTrue(datalink.facility_list.prev == &memory.facility.list);
    }

    TEST_METHOD(AppendRejectsReservedInfoAndDoubleRegistration)
    {
        gmp_datalink_t datalink{};
        fake_facility_t reserved{};
        fake_facility_t valid{};
        gmp_dev_dl_init(&datalink);
        init_fake(reserved, GMP_DL_FACILITY_USER, GMP_DL_CMD_INFO, 1U);
        init_fake(valid, GMP_DL_FACILITY_USER, 0x70U, 1U);

        Assert::AreEqual<fast_gt>(
            0, gmp_dev_dl_append_facility(&datalink, &reserved.facility));
        Assert::AreEqual<fast_gt>(
            1, gmp_dev_dl_append_facility(&datalink, &valid.facility));
        Assert::AreEqual<fast_gt>(
            0, gmp_dev_dl_append_facility(&datalink, &valid.facility));
    }

    TEST_METHOD(DispatcherRoutesByCommandAndCountsServiceRuns)
    {
        gmp_datalink_t datalink{};
        fake_facility_t service{};
        gmp_dev_dl_init(&datalink);
        init_fake(service, GMP_DL_FACILITY_USER, 0x70U, 2U);
        Assert::AreEqual<fast_gt>(
            1, gmp_dev_dl_append_facility(&datalink, &service.facility));

        datalink.rx_head.seq_id = 9U;
        datalink.rx_head.cmd = 0x71U;
        Assert::AreEqual<fast_gt>(1, gmp_dev_dl_dispatch_rx(&datalink));
        Assert::AreEqual<uint32_t>(1U, service.calls);
        Assert::AreEqual<uint16_t>(0x71U, datalink.tx_head.cmd);
        Assert::AreEqual<int>((int)GMP_DL_TX_STATE_READY_TO_WARP,
                              (int)datalink.tx_state);

        gmp_dev_dl_tx_state_done(&datalink);
        (void)gmp_dev_dl_loop_cb(&datalink);
        Assert::AreEqual<uint32_t>(1U, (uint32_t)datalink.service_run_count);
    }

    TEST_METHOD(InfoIsGeneratedFromRegisteredFacilities)
    {
        gmp_datalink_t datalink{};
        fake_facility_t tunable{};
        fake_facility_t scope{};
        gmp_dev_dl_init(&datalink);
        init_fake(tunable, GMP_DL_FACILITY_TUNABLE, 0x30U, 2U);
        init_fake(scope, GMP_DL_FACILITY_SCOPE, 0x60U, 1U);
        gmp_dev_dl_append_facility(&datalink, &tunable.facility);
        gmp_dev_dl_append_facility(&datalink, &scope.facility);

        datalink.rx_head.seq_id = 3U;
        datalink.rx_head.cmd = GMP_DL_CMD_INFO;
        Assert::AreEqual<fast_gt>(1, gmp_dev_dl_dispatch_rx(&datalink));
        Assert::AreEqual<size_gt>(11U, datalink.tx_len);
        Assert::AreEqual<byte_gt>(GMP_DL_INFO_PROTOCOL_VERSION,
                                  datalink.tx_buf[0]);
        Assert::AreEqual<byte_gt>(2U, datalink.tx_buf[4]);
        Assert::AreEqual<byte_gt>(GMP_DL_FACILITY_TUNABLE,
                                  datalink.tx_buf[5]);
        Assert::AreEqual<byte_gt>(0x30U, datalink.tx_buf[6]);
        Assert::AreEqual<byte_gt>(2U, datalink.tx_buf[7]);
        Assert::AreEqual<byte_gt>(GMP_DL_FACILITY_SCOPE,
                                  datalink.tx_buf[8]);
        Assert::AreEqual<byte_gt>(0x60U, datalink.tx_buf[9]);
        Assert::AreEqual<byte_gt>(1U, datalink.tx_buf[10]);
    }

    TEST_METHOD(RemoveDetachesOnlyRegisteredFacility)
    {
        gmp_datalink_t datalink{};
        fake_facility_t first{};
        fake_facility_t second{};
        gmp_dev_dl_init(&datalink);
        init_fake(first, GMP_DL_FACILITY_USER, 0x70U, 1U);
        init_fake(second, GMP_DL_FACILITY_USER, 0x71U, 1U);
        gmp_dev_dl_append_facility(&datalink, &first.facility);

        Assert::AreEqual<fast_gt>(
            0, gmp_dev_dl_remove_facility(&datalink, &second.facility));
        Assert::AreEqual<fast_gt>(
            1, gmp_dev_dl_remove_facility(&datalink, &first.facility));
        Assert::AreEqual<uint16_t>(0U, datalink.facility_count);
        Assert::AreEqual<fast_gt>(1, gmp_list_is_detached(&first.facility.list));
        Assert::AreEqual<fast_gt>(1, gmp_list_is_empty(&datalink.facility_list));
    }

    TEST_METHOD(EchoAliasIsARegisteredUserFacility)
    {
        gmp_datalink_t datalink{};
        gmp_dl_facility_t alias{};
        gmp_dev_dl_init(&datalink);
        gmp_dev_dl_init_echo_alias(&alias, 0x99U);
        Assert::AreEqual<fast_gt>(
            1, gmp_dev_dl_append_facility(&datalink, &alias));
        Assert::AreEqual<int>((int)GMP_DL_FACILITY_USER, (int)alias.type);

        datalink.rx_head.seq_id = 8U;
        datalink.rx_head.cmd = 0x99U;
        datalink.expected_payload_len = 3U;
        datalink.payload_buf[0] = 0x12U;
        datalink.payload_buf[1] = 0x34U;
        datalink.payload_buf[2] = 0x56U;
        Assert::AreEqual<fast_gt>(1, gmp_dev_dl_dispatch_rx(&datalink));
        Assert::AreEqual<uint16_t>(GMP_DL_CMD_ECHO, datalink.tx_head.cmd);
        Assert::AreEqual<size_gt>(3U, datalink.tx_len);
        Assert::AreEqual<byte_gt>(0x12U, datalink.tx_buf[0]);
        Assert::AreEqual<byte_gt>(0x34U, datalink.tx_buf[1]);
        Assert::AreEqual<byte_gt>(0x56U, datalink.tx_buf[2]);
    }

    TEST_METHOD(DispatcherNacksFacilityCountCorruption)
    {
        gmp_datalink_t datalink{};
        fake_facility_t service{};
        gmp_dev_dl_init(&datalink);
        init_fake(service, GMP_DL_FACILITY_USER, 0x70U, 1U);
        Assert::AreEqual<fast_gt>(
            1, gmp_dev_dl_append_facility(&datalink, &service.facility));

        datalink.facility_count = 0U;
        datalink.rx_head.seq_id = 4U;
        datalink.rx_head.cmd = GMP_DL_CMD_INFO;
        Assert::AreEqual<fast_gt>(1, gmp_dev_dl_dispatch_rx(&datalink));
        Assert::AreEqual<uint16_t>(GMP_DL_CMD_NACK, datalink.tx_head.cmd);
        Assert::AreEqual<fast_gt>(
            0, gmp_dev_dl_append_facility(&datalink, &service.facility));
    }
};
} // namespace gmp_core_unit_test
