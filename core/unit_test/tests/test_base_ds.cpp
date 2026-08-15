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
        gmp_list head{};
        list_item_t first{{}, 10};
        list_item_t second{{}, 20};

        gmp_list_init(&head);
        gmp_list_init(&first.link);
        gmp_list_init(&second.link);
        Assert::AreEqual<fast_gt>(1, gmp_list_push_tail(&head, &first.link));
        Assert::AreEqual<fast_gt>(1, gmp_list_push_tail(&head, &second.link));
        Assert::AreEqual<size_gt>(2, gmp_list_count(&head, 2));
        Assert::AreEqual(10, GMP_CONTAINER_OF(head.next, list_item_t, link)->value);
        Assert::AreEqual(20, GMP_CONTAINER_OF(head.prev, list_item_t, link)->value);
    }

    TEST_METHOD(IntrusiveListSupportsHeadTailAndMiddleInsertion)
    {
        gmp_list head{};
        list_item_t first{{}, 1};
        list_item_t second{{}, 2};
        list_item_t third{{}, 3};
        gmp_list_init(&head);
        gmp_list_init(&first.link);
        gmp_list_init(&second.link);
        gmp_list_init(&third.link);

        Assert::AreEqual<fast_gt>(1, gmp_list_push_head(&head, &second.link));
        Assert::AreEqual<fast_gt>(1, gmp_list_insert_before(&second.link, &first.link));
        Assert::AreEqual<fast_gt>(1, gmp_list_insert_after(&second.link, &third.link));
        Assert::AreEqual(1, GMP_CONTAINER_OF(head.next, list_item_t, link)->value);
        Assert::AreEqual(2, GMP_CONTAINER_OF(head.next->next, list_item_t, link)->value);
        Assert::AreEqual(3, GMP_CONTAINER_OF(head.prev, list_item_t, link)->value);
        Assert::AreEqual<fast_gt>(1, gmp_list_validate(&head, 3));
    }

    TEST_METHOD(IntrusiveListRejectsDoubleInsertAndDoubleRemove)
    {
        gmp_list head{};
        list_item_t item{{}, 7};
        gmp_list_init(&head);
        gmp_list_init(&item.link);

        Assert::AreEqual<fast_gt>(1, gmp_list_push_tail(&head, &item.link));
        Assert::AreEqual<fast_gt>(0, gmp_list_push_tail(&head, &item.link));
        Assert::AreEqual<fast_gt>(1, gmp_list_remove(&item.link));
        Assert::AreEqual<fast_gt>(0, gmp_list_remove(&item.link));
        Assert::AreEqual<fast_gt>(1, gmp_list_is_detached(&item.link));
        Assert::AreEqual<fast_gt>(1, gmp_list_is_empty(&head));
    }

    TEST_METHOD(IntrusiveListPopPreservesOrderAndDetachesEntries)
    {
        gmp_list head{};
        list_item_t first{{}, 1};
        list_item_t second{{}, 2};
        gmp_list_init(&head);
        gmp_list_init(&first.link);
        gmp_list_init(&second.link);
        gmp_list_push_tail(&head, &first.link);
        gmp_list_push_tail(&head, &second.link);

        Assert::IsTrue(gmp_list_pop_head(&head) == &first.link);
        Assert::AreEqual<fast_gt>(1, gmp_list_is_detached(&first.link));
        Assert::IsTrue(gmp_list_pop_tail(&head) == &second.link);
        Assert::AreEqual<fast_gt>(1, gmp_list_is_detached(&second.link));
        Assert::IsNull(gmp_list_pop_head(&head));
    }

    TEST_METHOD(IntrusiveListValidationDetectsCorruptionAndBoundsTraversal)
    {
        gmp_list head{};
        list_item_t first{{}, 1};
        list_item_t second{{}, 2};
        gmp_list_init(&head);
        gmp_list_init(&first.link);
        gmp_list_init(&second.link);
        gmp_list_push_tail(&head, &first.link);
        gmp_list_push_tail(&head, &second.link);

        Assert::AreEqual<fast_gt>(0, gmp_list_validate(&head, 1));
        Assert::AreEqual<size_gt>(2, gmp_list_count(&head, 2));
        second.link.prev = &head;
        Assert::AreEqual<fast_gt>(0, gmp_list_validate(&head, 2));
        Assert::AreEqual<size_gt>(3, gmp_list_count(&head, 2));
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
