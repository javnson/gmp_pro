/**
 * @file test_base_ds.cpp
 * @brief Visual Studio native tests for core/base data structures.
 */

#include "vs_test_support.h"

#include <core/base/ds/list.h>
#include <core/base/ds/ring_buf.h>

#include <cstring>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_test
{
namespace
{
struct list_item_t
{
    gmp_list link;
    int value;
};
} // namespace

TEST_CLASS(BaseDataStructureTests)
{
  public:
    TEST_METHOD(IntrusiveListLinksContainingObjects)
    {
        list_item_t first{{nullptr}, 10};
        list_item_t second{{nullptr}, 20};

        first.link.next = &second.link;
        Assert::IsTrue(first.link.next == &second.link);
        Assert::AreEqual(20, reinterpret_cast<list_item_t*>(first.link.next)->value);
    }

    TEST_METHOD(RingBufferReservesOneSlotAndPreservesFifoOrder)
    {
        byte_gt pool[4]{};
        byte_gt value = 0;
        ringbuf_t buffer{};

        ringbuf_init(&buffer, pool, sizeof(pool));
        Assert::AreEqual<size_gt>(0, ringbuf_used(&buffer));
        Assert::AreEqual<size_gt>(3, ringbuf_get_free(&buffer));
        Assert::AreEqual<fast_gt>(0, ringbuf_get_one(&buffer, &value));

        Assert::AreEqual<fast_gt>(1, ringbuf_put_one(&buffer, 1));
        Assert::AreEqual<fast_gt>(1, ringbuf_put_one(&buffer, 2));
        Assert::AreEqual<fast_gt>(1, ringbuf_put_one(&buffer, 3));
        Assert::AreEqual<fast_gt>(0, ringbuf_put_one(&buffer, 4));

        Assert::AreEqual<fast_gt>(1, ringbuf_get_one(&buffer, &value));
        Assert::AreEqual<byte_gt>(1, value);
        Assert::AreEqual<fast_gt>(1, ringbuf_put_one(&buffer, 4));

        for (const byte_gt expected : {static_cast<byte_gt>(2), static_cast<byte_gt>(3), static_cast<byte_gt>(4)})
        {
            Assert::AreEqual<fast_gt>(1, ringbuf_get_one(&buffer, &value));
            Assert::AreEqual<byte_gt>(expected, value);
        }
        Assert::AreEqual<fast_gt>(0, ringbuf_get_one(&buffer, &value));
    }

    TEST_METHOD(RingBufferArrayOperationsWrapAndClampToAvailableSpace)
    {
        static const byte_gt first_input[] = {10, 11, 12, 13};
        static const byte_gt second_input[] = {20, 21, 22, 23};
        static const byte_gt expected[] = {12, 13, 20, 21};
        byte_gt pool[5]{};
        byte_gt output[4]{};
        ringbuf_t buffer{};

        ringbuf_init(&buffer, pool, sizeof(pool));
        Assert::AreEqual<size_gt>(4, ringbuf_put_array(&buffer, first_input, sizeof(first_input)));
        Assert::AreEqual<size_gt>(2, ringbuf_get_array(&buffer, output, 2));
        Assert::AreEqual<size_gt>(2, ringbuf_put_array(&buffer, second_input, sizeof(second_input)));
        Assert::AreEqual<size_gt>(4, ringbuf_get_array(&buffer, output, sizeof(output)));
        Assert::IsTrue(std::memcmp(expected, output, sizeof(expected)) == 0);
        Assert::AreEqual<size_gt>(0, ringbuf_used(&buffer));
    }
};
} // namespace gmp_core_unit_test

