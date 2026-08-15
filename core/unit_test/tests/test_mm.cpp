/**
 * @file test_mm.cpp
 * @brief Visual Studio native tests for GMP block memory.
 */

#include "vs_test_support.h"

#include <core/mm/block_mem.h>

#include <cstring>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_test
{
namespace
{
union aligned_heap_t
{
    long double alignment;
    byte_gt bytes[1024];
};

gmp_mem_area_head* initialize_heap(aligned_heap_t& heap)
{
    std::memset(&heap, 0, sizeof(heap));
    return gmp_mm_setup_block_memory(heap.bytes, sizeof(heap.bytes), 32);
}
} // namespace

TEST_CLASS(BlockMemoryTests)
{
  public:
    TEST_METHOD(SetupInitializesCapacityAndStatus)
    {
        aligned_heap_t heap{};
        gmp_mem_area_head* area = initialize_heap(heap);

        Assert::IsNotNull(area);
        Assert::AreEqual<ec_gt>(GMP_EC_OK, gmp_mm_block_last_errors);
        Assert::AreEqual<size_gt>(32, area->capacity);
        Assert::IsTrue(area->used > 0);
    }

    TEST_METHOD(ReleasedBlocksAreReusableAndUsageReturnsToBaseline)
    {
        aligned_heap_t heap{};
        gmp_mem_area_head* area = initialize_heap(heap);
        Assert::IsNotNull(area);
        const size_gt baseline_usage = area->used;

        void* first = gmp_mm_block_alloc(area, 24);
        void* second = gmp_mm_block_alloc(area, 80);
        Assert::IsNotNull(first);
        Assert::IsNotNull(second);
        Assert::IsTrue(first != second);

        std::memset(first, 0xA5, 24);
        std::memset(second, 0x5A, 80);
        gmp_mm_block_free(area, first);
        Assert::AreEqual<ec_gt>(GMP_EC_OK, gmp_mm_block_last_errors);

        void* reused = gmp_mm_block_alloc(area, 24);
        Assert::IsTrue(reused == first);
        gmp_mm_block_free(area, reused);
        gmp_mm_block_free(area, second);
        Assert::AreEqual<size_gt>(baseline_usage, area->used);
    }

    TEST_METHOD(InvalidCapacityRequestsReportErrors)
    {
        aligned_heap_t heap{};
        gmp_mem_area_head* area = initialize_heap(heap);
        Assert::IsNotNull(area);

        Assert::IsNull(gmp_mm_block_alloc(area, sizeof(heap.bytes)));
        Assert::AreEqual<ec_gt>(GMP_EC_MM_NOT_ENOUGH_MEM, gmp_mm_block_last_errors);
        Assert::IsNull(gmp_mm_setup_block_memory(heap.bytes, sizeof(gmp_mem_area_head), 32));
        Assert::AreEqual<ec_gt>(GMP_EC_MM_NOT_ENOUGH_MEM, gmp_mm_block_last_errors);
    }
};
} // namespace gmp_core_unit_test

