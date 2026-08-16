/** @file test_rbtree.cpp @brief Strict intrusive red-black tree tests. */

#include "vs_test_support.h"

#include <core/base/ds/rbtree.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_test
{
namespace
{
struct rb_item_t
{
    gmp_rb_node node;
    int key;
};

int compare_items(const gmp_rb_node* lhs, const gmp_rb_node* rhs, void*)
{
    const auto* a = GMP_CONTAINER_OF(lhs, rb_item_t, node);
    const auto* b = GMP_CONTAINER_OF(rhs, rb_item_t, node);
    return a->key < b->key ? -1 : (a->key > b->key ? 1 : 0);
}

int compare_key(const void* key, const gmp_rb_node* node, void*)
{
    const int wanted = *static_cast<const int*>(key);
    const int present = GMP_CONTAINER_OF(node, rb_item_t, node)->key;
    return wanted < present ? -1 : (wanted > present ? 1 : 0);
}
} // namespace

TEST_CLASS(RedBlackTreeTests)
{
  public:
    TEST_METHOD(InsertFindAndTraversalRemainOrdered)
    {
        static const int order[] = {31, 7, 45, 2, 18, 39, 63, 1, 5, 12, 24, 35, 42, 52, 70};
        rb_item_t items[71]{};
        gmp_rb_root tree{};
        gmp_rb_root_init(&tree);
        for (const int key : order)
        {
            items[key].key = key;
            gmp_rb_node_init(&items[key].node);
            Assert::AreEqual<fast_gt>(1, gmp_rb_insert(&tree, &items[key].node,
                                                        compare_items, nullptr));
            Assert::AreEqual<fast_gt>(1, gmp_rb_validate(&tree, compare_items,
                                                          nullptr, 71));
        }
        int expected = 0;
        for (gmp_rb_node* node = gmp_rb_minimum(tree.root); node != nullptr;
             node = gmp_rb_next(node))
        {
            const int key = GMP_CONTAINER_OF(node, rb_item_t, node)->key;
            while (expected < key)
                ++expected;
            Assert::AreEqual(expected, key);
            ++expected;
        }
        for (const int key : order)
            Assert::IsTrue(gmp_rb_find(&tree, &key, compare_key, nullptr) ==
                           &items[key].node);
    }

    TEST_METHOD(DeleteEveryStructuralCasePreservesInvariants)
    {
        rb_item_t items[64]{};
        gmp_rb_root tree{};
        gmp_rb_root_init(&tree);
        for (int key = 0; key < 64; ++key)
        {
            items[key].key = key;
            gmp_rb_node_init(&items[key].node);
            Assert::AreEqual<fast_gt>(1, gmp_rb_insert(&tree, &items[key].node,
                                                        compare_items, nullptr));
        }
        for (int phase = 0; phase < 2; ++phase)
        {
            for (int key = phase; key < 64; key += 2)
            {
                Assert::AreEqual<fast_gt>(1, gmp_rb_remove(&tree, &items[key].node));
                Assert::AreEqual<fast_gt>(1, gmp_rb_node_is_detached(&items[key].node));
                Assert::AreEqual<fast_gt>(1, gmp_rb_validate(&tree, compare_items,
                                                              nullptr, 64));
            }
        }
        Assert::IsNull(tree.root);
        Assert::AreEqual<size_gt>(0, tree.count);
    }

    TEST_METHOD(DuplicateAndDoubleOperationsAreRejected)
    {
        rb_item_t first{};
        rb_item_t duplicate{};
        gmp_rb_root tree{};
        first.key = duplicate.key = 10;
        gmp_rb_root_init(&tree);
        gmp_rb_node_init(&first.node);
        gmp_rb_node_init(&duplicate.node);
        Assert::AreEqual<fast_gt>(1, gmp_rb_insert(&tree, &first.node,
                                                    compare_items, nullptr));
        Assert::AreEqual<fast_gt>(0, gmp_rb_insert(&tree, &first.node,
                                                    compare_items, nullptr));
        Assert::AreEqual<fast_gt>(0, gmp_rb_insert(&tree, &duplicate.node,
                                                    compare_items, nullptr));
        Assert::AreEqual<fast_gt>(1, gmp_rb_remove(&tree, &first.node));
        Assert::AreEqual<fast_gt>(0, gmp_rb_remove(&tree, &first.node));
    }

    TEST_METHOD(RemoveRejectsNodeOwnedByAnotherTree)
    {
        rb_item_t first{};
        rb_item_t second{};
        gmp_rb_root first_tree{};
        gmp_rb_root second_tree{};
        first.key = 1;
        second.key = 2;
        gmp_rb_root_init(&first_tree);
        gmp_rb_root_init(&second_tree);
        gmp_rb_node_init(&first.node);
        gmp_rb_node_init(&second.node);
        Assert::AreEqual<fast_gt>(1, gmp_rb_insert(
            &first_tree, &first.node, compare_items, nullptr));
        Assert::AreEqual<fast_gt>(1, gmp_rb_insert(
            &second_tree, &second.node, compare_items, nullptr));
        Assert::AreEqual<fast_gt>(0, gmp_rb_remove(&second_tree, &first.node));
        Assert::AreEqual<fast_gt>(1, gmp_rb_validate(
            &first_tree, compare_items, nullptr, 1));
        Assert::AreEqual<fast_gt>(1, gmp_rb_validate(
            &second_tree, compare_items, nullptr, 1));
    }

    TEST_METHOD(DeterministicPermutationsStressInsertAndDeleteFixups)
    {
        constexpr int item_count = 127;
        rb_item_t items[item_count]{};
        gmp_rb_root tree{};
        gmp_rb_root_init(&tree);
        for (int ordinal = 0; ordinal < item_count; ++ordinal)
        {
            const int key = (ordinal * 53) % item_count;
            items[key].key = key;
            gmp_rb_node_init(&items[key].node);
            Assert::AreEqual<fast_gt>(1, gmp_rb_insert(
                &tree, &items[key].node, compare_items, nullptr));
            Assert::AreEqual<fast_gt>(1, gmp_rb_validate(
                &tree, compare_items, nullptr, item_count));
        }
        for (int ordinal = 0; ordinal < item_count; ++ordinal)
        {
            const int key = (ordinal * 89 + 17) % item_count;
            Assert::AreEqual<fast_gt>(1,
                gmp_rb_remove(&tree, &items[key].node));
            Assert::AreEqual<fast_gt>(1, gmp_rb_validate(
                &tree, compare_items, nullptr, item_count));
        }
        Assert::AreEqual<size_gt>(0, tree.count);
    }

    TEST_METHOD(ValidationDetectsColourAndParentCorruption)
    {
        rb_item_t items[3]{};
        gmp_rb_root tree{};
        gmp_rb_root_init(&tree);
        for (int key = 0; key < 3; ++key)
        {
            items[key].key = key;
            gmp_rb_node_init(&items[key].node);
            gmp_rb_insert(&tree, &items[key].node, compare_items, nullptr);
        }
        Assert::AreEqual<fast_gt>(1, gmp_rb_validate(&tree, compare_items, nullptr, 3));
        tree.root->color = GMP_RB_RED;
        Assert::AreEqual<fast_gt>(0, gmp_rb_validate(&tree, compare_items, nullptr, 3));
        tree.root->color = GMP_RB_BLACK;
        tree.root->left->parent = nullptr;
        Assert::AreEqual<fast_gt>(0, gmp_rb_validate(&tree, compare_items, nullptr, 3));
    }
};
} // namespace gmp_core_unit_test
