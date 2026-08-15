/**
 * @file list.h
 * @brief Allocation-free intrusive doubly linked list.
 *
 * The API follows the circular sentinel model used by the Windows NT
 * LIST_ENTRY helpers. Every head and every detached entry is self-linked.
 * Mutating helpers reject NULL, corrupt neighbours and double insertion.
 */

#ifndef _FILE_GMP_DATASTRUCT_LIST_H_
#define _FILE_GMP_DATASTRUCT_LIST_H_

#include <gmp_type.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Intrusive list entry and circular list-head type. */
typedef struct _tag_gmp_list
{
    struct _tag_gmp_list* next;
    struct _tag_gmp_list* prev;
} gmp_list;

/** @brief Recover an owner object from one of its intrusive list members. */
#define GMP_CONTAINER_OF(entry, type, member) \
    ((type*)((byte_gt*)(entry) - offsetof(type, member)))

/** @brief Initialize a list head or reset a detached entry. */
GMP_STATIC_INLINE void gmp_list_init(gmp_list* entry)
{
    if (entry != NULL)
    {
        entry->next = entry;
        entry->prev = entry;
    }
}

/** @brief Return nonzero when a head contains no entries. */
GMP_STATIC_INLINE fast_gt gmp_list_is_empty(const gmp_list* head)
{
    return (head != NULL && head->next == head && head->prev == head) ? 1 : 0;
}

/** @brief Return nonzero when an entry is initialized and not linked. */
GMP_STATIC_INLINE fast_gt gmp_list_is_detached(const gmp_list* entry)
{
    return gmp_list_is_empty(entry);
}

/** @brief Return nonzero when the immediate neighbours form a valid chain. */
GMP_STATIC_INLINE fast_gt gmp_list_links_valid(const gmp_list* entry)
{
    return (entry != NULL && entry->next != NULL && entry->prev != NULL &&
            entry->next->prev == entry && entry->prev->next == entry) ? 1 : 0;
}

/** @brief Insert a detached entry immediately after @p position. */
GMP_STATIC_INLINE fast_gt gmp_list_insert_after(gmp_list* position,
                                                gmp_list* entry)
{
    gmp_list* next;
    if (!gmp_list_links_valid(position) || !gmp_list_is_detached(entry) ||
        position == entry)
        return 0;

    next = position->next;
    entry->prev = position;
    entry->next = next;
    position->next = entry;
    next->prev = entry;
    return 1;
}

/** @brief Insert a detached entry immediately before @p position. */
GMP_STATIC_INLINE fast_gt gmp_list_insert_before(gmp_list* position,
                                                 gmp_list* entry)
{
    gmp_list* previous;
    if (!gmp_list_links_valid(position) || !gmp_list_is_detached(entry) ||
        position == entry)
        return 0;

    previous = position->prev;
    entry->next = position;
    entry->prev = previous;
    previous->next = entry;
    position->prev = entry;
    return 1;
}

/** @brief Insert a detached entry at the front of a list. */
GMP_STATIC_INLINE fast_gt gmp_list_push_head(gmp_list* head, gmp_list* entry)
{
    return gmp_list_insert_after(head, entry);
}

/** @brief Insert a detached entry at the back of a list. */
GMP_STATIC_INLINE fast_gt gmp_list_push_tail(gmp_list* head, gmp_list* entry)
{
    return gmp_list_insert_before(head, entry);
}

/** @brief Remove a linked entry and reset it to the detached state. */
GMP_STATIC_INLINE fast_gt gmp_list_remove(gmp_list* entry)
{
    gmp_list* next;
    gmp_list* previous;
    if (!gmp_list_links_valid(entry) || gmp_list_is_detached(entry))
        return 0;

    next = entry->next;
    previous = entry->prev;
    previous->next = next;
    next->prev = previous;
    gmp_list_init(entry);
    return 1;
}

/** @brief Remove and return the first entry, or NULL for an empty list. */
GMP_STATIC_INLINE gmp_list* gmp_list_pop_head(gmp_list* head)
{
    gmp_list* entry;
    if (!gmp_list_links_valid(head) || gmp_list_is_empty(head))
        return NULL;
    entry = head->next;
    return gmp_list_remove(entry) ? entry : NULL;
}

/** @brief Remove and return the final entry, or NULL for an empty list. */
GMP_STATIC_INLINE gmp_list* gmp_list_pop_tail(gmp_list* head)
{
    gmp_list* entry;
    if (!gmp_list_links_valid(head) || gmp_list_is_empty(head))
        return NULL;
    entry = head->prev;
    return gmp_list_remove(entry) ? entry : NULL;
}

/**
 * @brief Validate a complete circular list without unbounded traversal.
 * @param head Circular sentinel head.
 * @param max_entries Maximum non-head entries allowed during the audit.
 * @return Nonzero only when every forward/backward link is coherent and the
 *         traversal returns to @p head within @p max_entries entries.
 */
GMP_STATIC_INLINE fast_gt gmp_list_validate(const gmp_list* head,
                                            size_gt max_entries)
{
    const gmp_list* current;
    size_gt count = 0U;
    if (!gmp_list_links_valid(head))
        return 0;

    current = head->next;
    while (current != head)
    {
        if (count >= max_entries || !gmp_list_links_valid(current))
            return 0;
        current = current->next;
        ++count;
    }
    return 1;
}

/** @brief Count entries, returning @p max_entries + 1 when invalid/too long. */
GMP_STATIC_INLINE size_gt gmp_list_count(const gmp_list* head,
                                         size_gt max_entries)
{
    const gmp_list* current;
    size_gt count = 0U;
    if (!gmp_list_links_valid(head))
        return max_entries + 1U;

    current = head->next;
    while (current != head)
    {
        if (count >= max_entries || !gmp_list_links_valid(current))
            return max_entries + 1U;
        current = current->next;
        ++count;
    }
    return count;
}

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_DATASTRUCT_LIST_H_ */
