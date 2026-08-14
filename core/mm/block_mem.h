/**
 * @file block_mem.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// Memory Management of Block
#ifndef _FILE_BLOCK_MEM_H_
#define _FILE_BLOCK_MEM_H_

#include <gmp_type.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// const uint16_t area_head_size = sizeof(gmp_mem_area_head)/ block_size_unit

typedef struct _tag_gmp_mem_block_head
{
    uint_least16_t magic_number; // magic number
    size_gt block_index;         // the first block index
    size_gt block_size;          // per block_size_unit
    // uint32_t block_label;
    struct _tag_gmp_mem_block_head *next;
} gmp_mem_block_head;

typedef struct _tag_gmp_mem_area_head
{
    void *entry;             // memory entry
    size_gt block_size_unit; // block size per sizeof
    size_gt capacity;        // memory are separated into these parts,per block_size_unit
    size_gt used;            // per block_size_unit
    uint32_t memory_state;   // memory state
    //	data_gt* used_flag;		// memory block used flag
    struct _tag_gmp_mem_area_head *next;  // next area
    struct _tag_gmp_mem_block_head *head; // the first item of mem-block head
    data_gt assigned_flag;
} gmp_mem_area_head;

// global variables
extern ec_gt gmp_mm_block_last_errors;

// interface
gmp_mem_area_head *gmp_mm_setup_block_memory( // return the memory area handle
    void *memory_entry,
    uint32_t memory_size,   // bytes
    size_gt block_size_unit // = GMP_MEM_BLOCK_SIZE
);

// alloc function
void *gmp_mm_block_alloc(gmp_mem_area_head *handle, size_gt length);

void gmp_mm_block_free(gmp_mem_area_head *handle, void *ptr);

#if SPECIFY_GMP_DEFAULT_ALLOC == USING_GMP_BLOCK_DEFAULT_FUNCTION

// default alloc function
extern gmp_mem_area_head *default_gmp_area_mem_handle;

#endif // SPECIFY_GMP_DEFAULT_ALLOC

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BLOCK_MEM_H_
