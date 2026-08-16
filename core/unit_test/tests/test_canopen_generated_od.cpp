#include "CppUnitTest.h"

#include <core/protocol/canopen/cia301/gmp_cia301_od.h>
#include <core/protocol/canopen/cia401/gmp_cia401_od.h>
#include <core/protocol/canopen/cia402/gmp_cia402_od.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_tests
{
TEST_CLASS(canopen_generated_od_tests)
{
public:
    TEST_METHOD(profile_seeds_build_valid_independent_dictionaries)
    {
        gmp_canopen_od_t cia301 = {};
        gmp_canopen_od_t cia401 = {};
        gmp_canopen_od_t cia402 = {};

        Assert::AreEqual<fast_gt>(1, gmp_cia301_od_init(&cia301));
        Assert::AreEqual<fast_gt>(1, gmp_cia401_od_init(&cia401));
        Assert::AreEqual<fast_gt>(1, gmp_cia402_od_init(&cia402));
        Assert::AreEqual<std::size_t>(GMP_CIA301_OD_ENTRY_COUNT, cia301.entries.count);
        Assert::AreEqual<std::size_t>(GMP_CIA401_OD_ENTRY_COUNT, cia401.entries.count);
        Assert::AreEqual<std::size_t>(GMP_CIA402_OD_ENTRY_COUNT, cia402.entries.count);
        Assert::IsNotNull(gmp_canopen_od_find(&cia301, 0x1017U, 0U));
        Assert::IsNotNull(gmp_canopen_od_find(&cia401, 0x6401U, 1U));
        Assert::IsNotNull(gmp_canopen_od_find(&cia402, 0x6040U, 0U));
        Assert::IsNotNull(gmp_canopen_od_find(&cia402, 0x60FFU, 0U));

        gmp_canopen_od_t second_dictionary = {};
        Assert::AreEqual<fast_gt>(0, gmp_cia301_od_init(&second_dictionary));
        Assert::AreEqual<std::size_t>(0, second_dictionary.entries.count);
    }
};
} // namespace gmp_core_unit_tests
