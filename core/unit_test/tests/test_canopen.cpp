/** @file test_canopen.cpp @brief Frame-level CANopen core tests. */

#include "vs_test_support.h"

#include <core/protocol/canopen/nmt_sm.h>
#include <core/protocol/canopen/pdo_engine.h>
#include <core/protocol/canopen/sdo_engine.h>

#include <cstring>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_test
{
namespace
{
void make_sdo_request(gmp_canopen_frame_t& frame, uint16_t node_id,
                      uint16_t command, uint16_t index, uint16_t subindex)
{
    gmp_canopen_frame_clear(&frame);
    frame.id = GMP_CANOPEN_COB_RSDO + node_id;
    frame.dlc = 8;
    frame.data[0] = (gmp_canopen_octet_t)command;
    gmp_canopen_store_le16(&frame.data[1], index);
    frame.data[3] = (gmp_canopen_octet_t)subindex;
}

struct send_capture_t
{
    uint16_t calls;
    gmp_canopen_frame_t frame;
};

fast_gt capture_send(void* context, const gmp_canopen_frame_t* frame)
{
    auto* capture = static_cast<send_capture_t*>(context);
    if (capture == nullptr || frame == nullptr)
        return 0;
    ++capture->calls;
    capture->frame = *frame;
    return 1;
}
} // namespace

TEST_CLASS(CanopenCoreTests)
{
  public:
    TEST_METHOD(ObjectDictionarySupportsValueAndPointerStorage)
    {
        gmp_canopen_od_t od{};
        gmp_canopen_od_entry_t value_entry{};
        gmp_canopen_od_entry_t pointer_entry{};
        gmp_canopen_od_entry_t invalid_subindex{};
        uint16_t pointer_value = 0x1234;
        gmp_canopen_octet_t bytes[4]{};
        uint32_t size = 0;
        gmp_canopen_od_init(&od);
        gmp_canopen_od_entry_init_value(&value_entry, 0x2000, 0,
            GMP_CANOPEN_OD_UNSIGNED32,
            GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, 0, "Value");
        value_entry.storage.value.u32 = 0x78563412UL;
        gmp_canopen_od_entry_init_pointer(&pointer_entry, 0x2001, 0,
            GMP_CANOPEN_OD_UNSIGNED16,
            GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE,
            &pointer_value, 0, "Pointer");
        Assert::AreEqual((int)GMP_CANOPEN_OD_OK, (int)gmp_canopen_od_insert(&od, &pointer_entry));
        Assert::AreEqual((int)GMP_CANOPEN_OD_OK, (int)gmp_canopen_od_insert(&od, &value_entry));
        gmp_canopen_od_entry_init_value(&invalid_subindex, 0x2002, 0x100,
            GMP_CANOPEN_OD_UNSIGNED8, GMP_CANOPEN_OD_ACCESS_READ,
            0, "Invalid sub-index");
        Assert::AreEqual((int)GMP_CANOPEN_OD_INVALID,
            (int)gmp_canopen_od_insert(&od, &invalid_subindex));
        Assert::AreEqual<fast_gt>(1, gmp_canopen_od_validate(&od, 2));
        Assert::IsTrue(gmp_canopen_od_find(&od, 0x2001, 0) == &pointer_entry);
        Assert::AreEqual((int)GMP_CANOPEN_OD_OK,
                         (int)gmp_canopen_od_read(&value_entry, bytes, 4, &size));
        Assert::AreEqual<uint32_t>(4, size);
        Assert::AreEqual<int>(0x12, bytes[0]);
        Assert::AreEqual<int>(0x78, bytes[3]);
        bytes[0] = 0xCD;
        bytes[1] = 0xAB;
        Assert::AreEqual((int)GMP_CANOPEN_OD_OK,
                         (int)gmp_canopen_od_write(&pointer_entry, bytes, 2));
        Assert::AreEqual<uint16_t>(0xABCD, pointer_value);
    }

    TEST_METHOD(NmtCommandsBootupAndHeartbeatFollowFrameContract)
    {
        gmp_canopen_nmt_t nmt{};
        gmp_canopen_frame_t frame{};
        Assert::AreEqual<fast_gt>(1, gmp_canopen_nmt_init(&nmt, 5, 100));
        Assert::AreEqual<fast_gt>(1, gmp_canopen_nmt_tick(&nmt, 0, &frame));
        Assert::AreEqual<uint32_t>(0x705, frame.id);
        Assert::AreEqual<int>(0, frame.data[0]);
        Assert::AreEqual((int)GMP_CANOPEN_NMT_PRE_OPERATIONAL, (int)nmt.state);

        gmp_canopen_frame_clear(&frame);
        frame.id = GMP_CANOPEN_COB_NMT;
        frame.dlc = 2;
        frame.data[0] = GMP_CANOPEN_NMT_CMD_START;
        frame.data[1] = 5;
        Assert::AreEqual((int)GMP_CANOPEN_NMT_EVENT_STATE_CHANGED,
                         (int)gmp_canopen_nmt_receive(&nmt, &frame));
        Assert::AreEqual((int)GMP_CANOPEN_NMT_OPERATIONAL, (int)nmt.state);
        frame.data[0] = GMP_CANOPEN_NMT_CMD_STOP;
        frame.data[1] = 0x80;
        Assert::AreEqual((int)GMP_CANOPEN_NMT_EVENT_NONE,
                         (int)gmp_canopen_nmt_receive(&nmt, &frame));
        Assert::AreEqual((int)GMP_CANOPEN_NMT_OPERATIONAL, (int)nmt.state);
        Assert::AreEqual<fast_gt>(0, gmp_canopen_nmt_tick(&nmt, 99, &frame));
        Assert::AreEqual<fast_gt>(1, gmp_canopen_nmt_tick(&nmt, 1, &frame));
        Assert::AreEqual<int>(GMP_CANOPEN_NMT_OPERATIONAL, frame.data[0]);
    }

    TEST_METHOD(HeartbeatConsumerTracksReceptionAndTimeout)
    {
        gmp_canopen_heartbeat_consumer_t consumer{};
        gmp_canopen_frame_t frame{};
        Assert::AreEqual<fast_gt>(1,
            gmp_canopen_heartbeat_consumer_init(&consumer, 7, 250));
        gmp_canopen_frame_clear(&frame);
        frame.id = 0x707;
        frame.dlc = 1;
        frame.data[0] = GMP_CANOPEN_NMT_OPERATIONAL;
        Assert::AreEqual<fast_gt>(1,
            gmp_canopen_heartbeat_consumer_receive(&consumer, &frame));
        Assert::AreEqual<fast_gt>(0,
            gmp_canopen_heartbeat_consumer_tick(&consumer, 249));
        Assert::AreEqual<fast_gt>(1,
            gmp_canopen_heartbeat_consumer_tick(&consumer, 1));
    }

    TEST_METHOD(SdoExpeditedUploadDownloadAndAbortWork)
    {
        gmp_canopen_od_t od{};
        gmp_canopen_od_entry_t rw{};
        gmp_canopen_od_entry_t ro{};
        gmp_canopen_sdo_server_t server{};
        gmp_canopen_frame_t request{};
        gmp_canopen_frame_t response{};
        gmp_canopen_od_init(&od);
        gmp_canopen_od_entry_init_value(&rw, 0x2000, 0, GMP_CANOPEN_OD_UNSIGNED32,
            GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, 0, "RW");
        gmp_canopen_od_entry_init_value(&ro, 0x2001, 0, GMP_CANOPEN_OD_UNSIGNED16,
            GMP_CANOPEN_OD_ACCESS_READ, 0, "RO");
        gmp_canopen_od_insert(&od, &rw);
        gmp_canopen_od_insert(&od, &ro);
        gmp_canopen_sdo_server_init(&server, 3, &od);

        make_sdo_request(request, 3, 0x23, 0x2000, 0);
        gmp_canopen_store_le32(&request.data[4], 0xDEADBEEFUL);
        Assert::AreEqual<fast_gt>(1,
            gmp_canopen_sdo_server_process(&server, &request, &response));
        Assert::AreEqual<int>(0x60, response.data[0]);
        Assert::AreEqual<uint32_t>(0xDEADBEEF, rw.storage.value.u32);

        make_sdo_request(request, 3, 0x40, 0x2000, 0);
        gmp_canopen_sdo_server_process(&server, &request, &response);
        Assert::AreEqual<int>(0x43, response.data[0]);
        Assert::AreEqual<uint32_t>(0xDEADBEEF, gmp_canopen_load_le32(&response.data[4]));

        make_sdo_request(request, 3, 0x2B, 0x2001, 0);
        request.data[4] = 1;
        gmp_canopen_sdo_server_process(&server, &request, &response);
        Assert::AreEqual<int>(0x80, response.data[0]);
        Assert::AreEqual<uint32_t>(GMP_CANOPEN_SDO_ABORT_READ_ONLY,
                                   gmp_canopen_load_le32(&response.data[4]));
    }

    TEST_METHOD(SdoSegmentedDownloadAndUploadPreservePayload)
    {
        gmp_canopen_od_t od{};
        gmp_canopen_od_entry_t domain{};
        gmp_canopen_sdo_server_t server{};
        gmp_canopen_frame_t request{};
        gmp_canopen_frame_t response{};
        gmp_canopen_octet_t storage[10]{};
        const gmp_canopen_octet_t expected[10] = {0,1,2,3,4,5,6,7,8,9};
        gmp_canopen_od_init(&od);
        gmp_canopen_od_entry_init_pointer(&domain, 0x2100, 0, GMP_CANOPEN_OD_DOMAIN,
            GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE,
            storage, 10, "Domain");
        gmp_canopen_od_insert(&od, &domain);
        gmp_canopen_sdo_server_init(&server, 9, &od);

        make_sdo_request(request, 9, 0x21, 0x2100, 0);
        gmp_canopen_store_le32(&request.data[4], 10);
        gmp_canopen_sdo_server_process(&server, &request, &response);
        Assert::AreEqual<int>(0x60, response.data[0]);
        make_sdo_request(request, 9, 0x00, 0, 0);
        for (int i = 0; i < 7; ++i) request.data[i + 1] = expected[i];
        gmp_canopen_sdo_server_process(&server, &request, &response);
        Assert::AreEqual<int>(0x20, response.data[0]);
        make_sdo_request(request, 9, 0x19, 0, 0);
        for (int i = 0; i < 3; ++i) request.data[i + 1] = expected[i + 7];
        gmp_canopen_sdo_server_process(&server, &request, &response);
        Assert::IsTrue(std::memcmp(storage, expected, 10) == 0);

        make_sdo_request(request, 9, 0x40, 0x2100, 0);
        gmp_canopen_sdo_server_process(&server, &request, &response);
        Assert::AreEqual<int>(0x41, response.data[0]);
        make_sdo_request(request, 9, 0x60, 0, 0);
        gmp_canopen_sdo_server_process(&server, &request, &response);
        Assert::AreEqual<int>(0x00, response.data[0]);
        for (int i = 0; i < 7; ++i) Assert::AreEqual<int>(expected[i], response.data[i + 1]);
        make_sdo_request(request, 9, 0x70, 0, 0);
        gmp_canopen_sdo_server_process(&server, &request, &response);
        Assert::AreEqual<int>(0x19, response.data[0]);
        for (int i = 0; i < 3; ++i) Assert::AreEqual<int>(expected[i + 7], response.data[i + 1]);
    }

    TEST_METHOD(SdoRejectsOutOfSequenceAndWrongToggleFrames)
    {
        gmp_canopen_od_t od{};
        gmp_canopen_od_entry_t domain{};
        gmp_canopen_sdo_server_t server{};
        gmp_canopen_frame_t request{};
        gmp_canopen_frame_t response{};
        gmp_canopen_octet_t storage[10]{};
        gmp_canopen_od_init(&od);
        gmp_canopen_od_entry_init_pointer(&domain, 0x2100, 2,
            GMP_CANOPEN_OD_DOMAIN,
            GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE,
            storage, 10, "Domain");
        Assert::AreEqual((int)GMP_CANOPEN_OD_OK,
            (int)gmp_canopen_od_insert(&od, &domain));
        Assert::AreEqual<fast_gt>(1,
            gmp_canopen_sdo_server_init(&server, 9, &od));

        make_sdo_request(request, 9, 0x60, 0, 0);
        Assert::AreEqual<fast_gt>(1,
            gmp_canopen_sdo_server_process(&server, &request, &response));
        Assert::AreEqual<uint32_t>(GMP_CANOPEN_SDO_ABORT_COMMAND,
            gmp_canopen_load_le32(&response.data[4]));

        make_sdo_request(request, 9, 0x21, 0x2100, 2);
        gmp_canopen_store_le32(&request.data[4], 10);
        gmp_canopen_sdo_server_process(&server, &request, &response);
        make_sdo_request(request, 9, 0x10, 0, 0);
        gmp_canopen_sdo_server_process(&server, &request, &response);
        Assert::AreEqual<uint32_t>(GMP_CANOPEN_SDO_ABORT_TOGGLE,
            gmp_canopen_load_le32(&response.data[4]));
        Assert::AreEqual<uint16_t>(0x2100, gmp_canopen_load_le16(&response.data[1]));
        Assert::AreEqual<int>(2, response.data[3]);
    }

    TEST_METHOD(SdoReceiveDispatchesExactlyOneResponseFrame)
    {
        gmp_canopen_od_t od{};
        gmp_canopen_od_entry_t entry{};
        gmp_canopen_sdo_server_t server{};
        gmp_canopen_frame_t request{};
        send_capture_t capture{};
        gmp_canopen_od_init(&od);
        gmp_canopen_od_entry_init_value(&entry, 0x2000, 0,
            GMP_CANOPEN_OD_UNSIGNED16, GMP_CANOPEN_OD_ACCESS_READ,
            0, "Read only");
        entry.storage.value.u16 = 0x1234;
        Assert::AreEqual((int)GMP_CANOPEN_OD_OK,
            (int)gmp_canopen_od_insert(&od, &entry));
        Assert::AreEqual<fast_gt>(1,
            gmp_canopen_sdo_server_init(&server, 3, &od));
        make_sdo_request(request, 3, 0x40, 0x2000, 0);
        Assert::AreEqual<fast_gt>(1, gmp_canopen_sdo_server_receive(
            &server, &request, capture_send, &capture));
        Assert::AreEqual<int>(1, capture.calls);
        Assert::AreEqual<uint32_t>(0x583, capture.frame.id);
        Assert::AreEqual<int>(0x4B, capture.frame.data[0]);
        Assert::AreEqual<uint16_t>(0x1234,
            gmp_canopen_load_le16(&capture.frame.data[4]));
    }

    TEST_METHOD(PdoCompileTransmitAndReceiveRequireOperationalState)
    {
        gmp_canopen_od_t od{};
        gmp_canopen_od_entry_t first{};
        gmp_canopen_od_entry_t second{};
        gmp_canopen_pdo_t tx{};
        gmp_canopen_pdo_t rx{};
        gmp_canopen_frame_t frame{};
        uint16_t first_value = 0x1234;
        uint32_t second_value = 0x89ABCDEF;
        const uint32_t maps[] = {
            GMP_CANOPEN_PDO_MAP(0x3000, 0, 16),
            GMP_CANOPEN_PDO_MAP(0x3001, 0, 32)};
        gmp_canopen_od_init(&od);
        gmp_canopen_od_entry_init_pointer(&first, 0x3000, 0, GMP_CANOPEN_OD_UNSIGNED16,
            GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE |
            GMP_CANOPEN_OD_ACCESS_PDO, &first_value, 0, "First");
        gmp_canopen_od_entry_init_pointer(&second, 0x3001, 0, GMP_CANOPEN_OD_UNSIGNED32,
            GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE |
            GMP_CANOPEN_OD_ACCESS_PDO, &second_value, 0, "Second");
        gmp_canopen_od_insert(&od, &first);
        gmp_canopen_od_insert(&od, &second);
        Assert::AreEqual((int)GMP_CANOPEN_PDO_OK,
            (int)gmp_canopen_pdo_compile(&tx, &od, GMP_CANOPEN_PDO_TRANSMIT,
                                         0x185, 255, maps, 2));
        Assert::AreEqual((int)GMP_CANOPEN_PDO_NOT_OPERATIONAL,
            (int)gmp_canopen_pdo_build(&tx, GMP_CANOPEN_NMT_PRE_OPERATIONAL, &frame));
        Assert::AreEqual((int)GMP_CANOPEN_PDO_OK,
            (int)gmp_canopen_pdo_build(&tx, GMP_CANOPEN_NMT_OPERATIONAL, &frame));
        Assert::AreEqual<int>(6, frame.dlc);
        Assert::AreEqual<uint16_t>(0x1234, gmp_canopen_load_le16(frame.data));
        Assert::AreEqual<uint32_t>(0x89ABCDEF, gmp_canopen_load_le32(&frame.data[2]));

        Assert::AreEqual((int)GMP_CANOPEN_PDO_OK,
            (int)gmp_canopen_pdo_compile(&rx, &od, GMP_CANOPEN_PDO_RECEIVE,
                                         0x205, 255, maps, 2));
        frame.id = 0x205;
        gmp_canopen_store_le16(frame.data, 0xBEEF);
        gmp_canopen_store_le32(&frame.data[2], 0x10203040);
        Assert::AreEqual((int)GMP_CANOPEN_PDO_OK,
            (int)gmp_canopen_pdo_apply(&rx, GMP_CANOPEN_NMT_OPERATIONAL, &frame));
        Assert::AreEqual<uint16_t>(0xBEEF, first_value);
        Assert::AreEqual<uint32_t>(0x10203040, second_value);
    }
};
} // namespace gmp_core_unit_test
