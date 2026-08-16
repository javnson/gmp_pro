/**
 * @file rbtree.h
 * @brief Allocation-free intrusive red-black tree with explicit metadata.
 *
 * Parent pointers and node colour are intentionally stored in separate
 * fields. Unlike the Linux kernel implementation, GMP does not pack colour
 * bits into a pointer because embedded targets do not share one guaranteed
 * pointer-alignment contract.
 */

#ifndef _FILE_GMP_DATASTRUCT_RBTREE_H_
#define _FILE_GMP_DATASTRUCT_RBTREE_H_

#include <gmp_type.h>
#include <stddef.h>

#ifndef GMP_CONTAINER_OF
#define GMP_CONTAINER_OF(entry, type, member) \
    ((type*)((byte_gt*)(entry) - offsetof(type, member)))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    GMP_RB_RED = 0,
    GMP_RB_BLACK = 1
} gmp_rb_color_t;

struct _tag_gmp_rb_root;

typedef struct _tag_gmp_rb_node
{
    struct _tag_gmp_rb_root* owner;
    struct _tag_gmp_rb_node* parent;
    struct _tag_gmp_rb_node* left;
    struct _tag_gmp_rb_node* right;
    gmp_rb_color_t color;
    fast_gt linked;
} gmp_rb_node;

typedef struct _tag_gmp_rb_root
{
    gmp_rb_node* root;
    size_gt count;
} gmp_rb_root;

typedef int (*gmp_rb_compare_fn)(const gmp_rb_node* lhs,
                                 const gmp_rb_node* rhs,
                                 void* context);
typedef int (*gmp_rb_key_compare_fn)(const void* key,
                                     const gmp_rb_node* node,
                                     void* context);

GMP_STATIC_INLINE void gmp_rb_node_init(gmp_rb_node* node)
{
    if (node != NULL)
    {
        node->parent = NULL;
        node->owner = NULL;
        node->left = NULL;
        node->right = NULL;
        node->color = GMP_RB_BLACK;
        node->linked = 0;
    }
}

GMP_STATIC_INLINE void gmp_rb_root_init(gmp_rb_root* tree)
{
    if (tree != NULL)
    {
        tree->root = NULL;
        tree->count = 0U;
    }
}

GMP_STATIC_INLINE fast_gt gmp_rb_node_is_detached(const gmp_rb_node* node)
{
    return (node != NULL && !node->linked && node->owner == NULL &&
            node->parent == NULL &&
            node->left == NULL && node->right == NULL) ? 1 : 0;
}

GMP_STATIC_INLINE gmp_rb_color_t gmp_rb_color(const gmp_rb_node* node)
{
    return node == NULL ? GMP_RB_BLACK : node->color;
}

GMP_STATIC_INLINE void gmp_rb_rotate_left(gmp_rb_root* tree,
                                           gmp_rb_node* node)
{
    gmp_rb_node* pivot = node->right;
    node->right = pivot->left;
    if (pivot->left != NULL)
        pivot->left->parent = node;
    pivot->parent = node->parent;
    if (node->parent == NULL)
        tree->root = pivot;
    else if (node == node->parent->left)
        node->parent->left = pivot;
    else
        node->parent->right = pivot;
    pivot->left = node;
    node->parent = pivot;
}

GMP_STATIC_INLINE void gmp_rb_rotate_right(gmp_rb_root* tree,
                                            gmp_rb_node* node)
{
    gmp_rb_node* pivot = node->left;
    node->left = pivot->right;
    if (pivot->right != NULL)
        pivot->right->parent = node;
    pivot->parent = node->parent;
    if (node->parent == NULL)
        tree->root = pivot;
    else if (node == node->parent->right)
        node->parent->right = pivot;
    else
        node->parent->left = pivot;
    pivot->right = node;
    node->parent = pivot;
}

GMP_STATIC_INLINE fast_gt gmp_rb_insert(gmp_rb_root* tree,
                                         gmp_rb_node* node,
                                         gmp_rb_compare_fn compare,
                                         void* context)
{
    gmp_rb_node* parent = NULL;
    gmp_rb_node* cursor;
    int order = 0;
    if (tree == NULL || node == NULL || compare == NULL ||
        !gmp_rb_node_is_detached(node))
        return 0;

    cursor = tree->root;
    while (cursor != NULL)
    {
        parent = cursor;
        order = compare(node, cursor, context);
        if (order == 0)
            return 0;
        cursor = order < 0 ? cursor->left : cursor->right;
    }

    node->parent = parent;
    node->owner = tree;
    node->color = GMP_RB_RED;
    node->linked = 1;
    if (parent == NULL)
        tree->root = node;
    else if (order < 0)
        parent->left = node;
    else
        parent->right = node;

    while (node != tree->root && gmp_rb_color(node->parent) == GMP_RB_RED)
    {
        gmp_rb_node* parent_node = node->parent;
        gmp_rb_node* grandparent = parent_node->parent;
        if (parent_node == grandparent->left)
        {
            gmp_rb_node* uncle = grandparent->right;
            if (gmp_rb_color(uncle) == GMP_RB_RED)
            {
                parent_node->color = GMP_RB_BLACK;
                uncle->color = GMP_RB_BLACK;
                grandparent->color = GMP_RB_RED;
                node = grandparent;
            }
            else
            {
                if (node == parent_node->right)
                {
                    node = parent_node;
                    gmp_rb_rotate_left(tree, node);
                    parent_node = node->parent;
                    grandparent = parent_node->parent;
                }
                parent_node->color = GMP_RB_BLACK;
                grandparent->color = GMP_RB_RED;
                gmp_rb_rotate_right(tree, grandparent);
            }
        }
        else
        {
            gmp_rb_node* uncle = grandparent->left;
            if (gmp_rb_color(uncle) == GMP_RB_RED)
            {
                parent_node->color = GMP_RB_BLACK;
                uncle->color = GMP_RB_BLACK;
                grandparent->color = GMP_RB_RED;
                node = grandparent;
            }
            else
            {
                if (node == parent_node->left)
                {
                    node = parent_node;
                    gmp_rb_rotate_right(tree, node);
                    parent_node = node->parent;
                    grandparent = parent_node->parent;
                }
                parent_node->color = GMP_RB_BLACK;
                grandparent->color = GMP_RB_RED;
                gmp_rb_rotate_left(tree, grandparent);
            }
        }
    }
    tree->root->color = GMP_RB_BLACK;
    tree->count++;
    return 1;
}

GMP_STATIC_INLINE gmp_rb_node* gmp_rb_minimum(gmp_rb_node* node)
{
    if (node == NULL)
        return NULL;
    while (node->left != NULL)
        node = node->left;
    return node;
}

GMP_STATIC_INLINE gmp_rb_node* gmp_rb_maximum(gmp_rb_node* node)
{
    if (node == NULL)
        return NULL;
    while (node->right != NULL)
        node = node->right;
    return node;
}

GMP_STATIC_INLINE gmp_rb_node* gmp_rb_next(gmp_rb_node* node)
{
    gmp_rb_node* parent;
    if (node == NULL || !node->linked)
        return NULL;
    if (node->right != NULL)
        return gmp_rb_minimum(node->right);
    parent = node->parent;
    while (parent != NULL && node == parent->right)
    {
        node = parent;
        parent = parent->parent;
    }
    return parent;
}

GMP_STATIC_INLINE gmp_rb_node* gmp_rb_previous(gmp_rb_node* node)
{
    gmp_rb_node* parent;
    if (node == NULL || !node->linked)
        return NULL;
    if (node->left != NULL)
        return gmp_rb_maximum(node->left);
    parent = node->parent;
    while (parent != NULL && node == parent->left)
    {
        node = parent;
        parent = parent->parent;
    }
    return parent;
}

GMP_STATIC_INLINE gmp_rb_node* gmp_rb_find(const gmp_rb_root* tree,
                                            const void* key,
                                            gmp_rb_key_compare_fn compare,
                                            void* context)
{
    gmp_rb_node* cursor;
    if (tree == NULL || key == NULL || compare == NULL)
        return NULL;
    cursor = tree->root;
    while (cursor != NULL)
    {
        int order = compare(key, cursor, context);
        if (order == 0)
            return cursor;
        cursor = order < 0 ? cursor->left : cursor->right;
    }
    return NULL;
}

GMP_STATIC_INLINE void gmp_rb_transplant(gmp_rb_root* tree,
                                          gmp_rb_node* old_node,
                                          gmp_rb_node* new_node)
{
    if (old_node->parent == NULL)
        tree->root = new_node;
    else if (old_node == old_node->parent->left)
        old_node->parent->left = new_node;
    else
        old_node->parent->right = new_node;
    if (new_node != NULL)
        new_node->parent = old_node->parent;
}

GMP_STATIC_INLINE void gmp_rb_delete_fixup(gmp_rb_root* tree,
                                            gmp_rb_node* node,
                                            gmp_rb_node* parent)
{
    while (node != tree->root && gmp_rb_color(node) == GMP_RB_BLACK)
    {
        gmp_rb_node* sibling;
        if (parent == NULL)
            break;
        if (node == parent->left)
        {
            sibling = parent->right;
            if (gmp_rb_color(sibling) == GMP_RB_RED)
            {
                sibling->color = GMP_RB_BLACK;
                parent->color = GMP_RB_RED;
                gmp_rb_rotate_left(tree, parent);
                sibling = parent->right;
            }
            if (sibling == NULL)
            {
                node = parent;
                parent = node->parent;
            }
            else if (gmp_rb_color(sibling->left) == GMP_RB_BLACK &&
                     gmp_rb_color(sibling->right) == GMP_RB_BLACK)
            {
                sibling->color = GMP_RB_RED;
                node = parent;
                parent = node->parent;
            }
            else
            {
                if (gmp_rb_color(sibling->right) == GMP_RB_BLACK)
                {
                    if (sibling->left != NULL)
                        sibling->left->color = GMP_RB_BLACK;
                    sibling->color = GMP_RB_RED;
                    gmp_rb_rotate_right(tree, sibling);
                    sibling = parent->right;
                }
                sibling->color = parent->color;
                parent->color = GMP_RB_BLACK;
                if (sibling->right != NULL)
                    sibling->right->color = GMP_RB_BLACK;
                gmp_rb_rotate_left(tree, parent);
                node = tree->root;
                parent = NULL;
            }
        }
        else
        {
            sibling = parent->left;
            if (gmp_rb_color(sibling) == GMP_RB_RED)
            {
                sibling->color = GMP_RB_BLACK;
                parent->color = GMP_RB_RED;
                gmp_rb_rotate_right(tree, parent);
                sibling = parent->left;
            }
            if (sibling == NULL)
            {
                node = parent;
                parent = node->parent;
            }
            else if (gmp_rb_color(sibling->right) == GMP_RB_BLACK &&
                     gmp_rb_color(sibling->left) == GMP_RB_BLACK)
            {
                sibling->color = GMP_RB_RED;
                node = parent;
                parent = node->parent;
            }
            else
            {
                if (gmp_rb_color(sibling->left) == GMP_RB_BLACK)
                {
                    if (sibling->right != NULL)
                        sibling->right->color = GMP_RB_BLACK;
                    sibling->color = GMP_RB_RED;
                    gmp_rb_rotate_left(tree, sibling);
                    sibling = parent->left;
                }
                sibling->color = parent->color;
                parent->color = GMP_RB_BLACK;
                if (sibling->left != NULL)
                    sibling->left->color = GMP_RB_BLACK;
                gmp_rb_rotate_right(tree, parent);
                node = tree->root;
                parent = NULL;
            }
        }
    }
    if (node != NULL)
        node->color = GMP_RB_BLACK;
}

GMP_STATIC_INLINE fast_gt gmp_rb_remove(gmp_rb_root* tree,
                                         gmp_rb_node* node)
{
    gmp_rb_node* moved;
    gmp_rb_node* fix_node;
    gmp_rb_node* fix_parent;
    gmp_rb_color_t original_color;
    if (tree == NULL || node == NULL || !node->linked ||
        node->owner != tree || tree->count == 0U)
        return 0;

    moved = node;
    original_color = moved->color;
    if (node->left == NULL)
    {
        fix_node = node->right;
        fix_parent = node->parent;
        gmp_rb_transplant(tree, node, node->right);
    }
    else if (node->right == NULL)
    {
        fix_node = node->left;
        fix_parent = node->parent;
        gmp_rb_transplant(tree, node, node->left);
    }
    else
    {
        moved = gmp_rb_minimum(node->right);
        original_color = moved->color;
        fix_node = moved->right;
        if (moved->parent == node)
        {
            fix_parent = moved;
            if (fix_node != NULL)
                fix_node->parent = moved;
        }
        else
        {
            fix_parent = moved->parent;
            gmp_rb_transplant(tree, moved, moved->right);
            moved->right = node->right;
            moved->right->parent = moved;
        }
        gmp_rb_transplant(tree, node, moved);
        moved->left = node->left;
        moved->left->parent = moved;
        moved->color = node->color;
    }

    if (original_color == GMP_RB_BLACK)
        gmp_rb_delete_fixup(tree, fix_node, fix_parent);
    tree->count--;
    gmp_rb_node_init(node);
    return 1;
}

GMP_STATIC_INLINE fast_gt gmp_rb_validate_subtree(
    const gmp_rb_root* tree, const gmp_rb_node* node,
    const gmp_rb_node* parent,
    const gmp_rb_node* lower, const gmp_rb_node* upper,
    gmp_rb_compare_fn compare, void* context, size_gt max_nodes,
    size_gt* visited, size_gt black_depth, size_gt* expected_black_depth)
{
    if (node == NULL)
    {
        if (*expected_black_depth == (size_gt)-1)
            *expected_black_depth = black_depth;
        return *expected_black_depth == black_depth ? 1 : 0;
    }
    if (*visited >= max_nodes || !node->linked || node->owner != tree ||
        node->parent != parent ||
        (lower != NULL && compare(node, lower, context) <= 0) ||
        (upper != NULL && compare(node, upper, context) >= 0) ||
        (node->color == GMP_RB_RED &&
         (gmp_rb_color(node->left) == GMP_RB_RED ||
          gmp_rb_color(node->right) == GMP_RB_RED)))
        return 0;
    (*visited)++;
    if (node->color == GMP_RB_BLACK)
        black_depth++;
    return gmp_rb_validate_subtree(tree, node->left, node, lower, node,
                                   compare, context, max_nodes, visited,
                                   black_depth, expected_black_depth) &&
           gmp_rb_validate_subtree(tree, node->right, node, node, upper,
                                   compare, context, max_nodes, visited,
                                   black_depth, expected_black_depth);
}

/** @brief Audit ordering, parent links, colours, black height, and count. */
GMP_STATIC_INLINE fast_gt gmp_rb_validate(const gmp_rb_root* tree,
                                           gmp_rb_compare_fn compare,
                                           void* context,
                                           size_gt max_nodes)
{
    size_gt visited = 0U;
    size_gt black_depth = (size_gt)-1;
    if (tree == NULL || compare == NULL || tree->count > max_nodes)
        return 0;
    if (tree->root == NULL)
        return tree->count == 0U ? 1 : 0;
    if (tree->root->parent != NULL || tree->root->color != GMP_RB_BLACK)
        return 0;
    if (!gmp_rb_validate_subtree(tree, tree->root, NULL, NULL, NULL,
                                 compare, context, max_nodes, &visited,
                                 0U, &black_depth))
        return 0;
    return visited == tree->count ? 1 : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_DATASTRUCT_RBTREE_H_ */
