/**
 * @file block_mem.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2024-09-30
 * 
 * @copyright Copyright GMP(c) 2024
 * 
 */

// This file provide a memory controller for the GMP
// The implementation of the MM is controlled by block.

#include <gmp_type.h>
#include <core/mm/block_mem.h>

#if defined SPECIFY_GMP_BLOCK_MEMORY_ENABLE

// global variables
ec_gt gmp_mm_block_last_errors = GMP_EC_OK;


// utilities

static void set_assigned_flag(gmp_mem_area_head* handle,
	size_gt position,
	size_gt length
) //GMP_NO_OPT
{
	byte_gt* assigned_flag = &handle->assigned_flag;

	size_gt first_item_index = position / GMP_PORT_DATA_SIZE_PER_BITS;
	size_gt first_item_subindex = position % GMP_PORT_DATA_SIZE_PER_BITS;

	size_gt last_item_index = (position + length) / GMP_PORT_DATA_SIZE_PER_BITS;
	size_gt last_item_subindex = (position + length) % GMP_PORT_DATA_SIZE_PER_BITS;

	size_gt i;

	// Deal with the first item
	for (i = first_item_subindex;
		i < last_item_subindex ||
		((first_item_index < last_item_index) && (i < GMP_PORT_DATA_SIZE_PER_BITS));
		++i)
		assigned_flag[first_item_index] |= 1 << i;

	if (first_item_index == last_item_index) // only the first item
		return;

	// Deal with the last item
	for (i = 0; i < last_item_subindex; ++i)
		assigned_flag[last_item_index] |= 1 << i;

	if (first_item_index == last_item_index - 1)
		return;

	// Deal with the medium items
	for (i = first_item_index + 1; i < last_item_index; ++i)
		assigned_flag[i] = (byte_gt)-1; // set all bits to 1

	return;
}

static void* fill_block_head(gmp_mem_area_head* handle,
	size_gt position,
	size_gt length
) //GMP_NO_OPT
{
	gmp_mem_block_head* block_head = (gmp_mem_block_head*)
		(((byte_gt*)handle->entry) + position * handle->block_size_unit);

	// clear the space
	memset((void*)block_head, 0, sizeof(gmp_mem_block_head));

	// fill the blank
	// form a stack structure
	block_head->block_size = length;
	block_head->block_index = position;
	block_head->magic_number = GMP_MEM_MAGIC_NUMBER;
	block_head->next = handle->head;

	handle->head = block_head;
	// fill the handle->assigned_flag
	set_assigned_flag(handle, position, length);

	// calculate the entrance
	// at the end of block
	return (void*)(block_head + 1);
}

// Setup the memory heap
gmp_mem_area_head* gmp_mm_setup_block_memory(	// return the memory area handle
	void* memory_entry,				// entry of the memory block
	uint32_t memory_size,		    // bytes
	size_gt block_size_unit
) //GMP_NO_OPT
{
	uint32_t memory_size_infimum = sizeof(gmp_mem_area_head) + sizeof(gmp_mem_block_head);
	size_gt capacity;
	size_gt bitmap_size;
	size_gt metadata_size;
	size_gt used;

	// not enough memory
	if (memory_entry == NULL || block_size_unit < sizeof(gmp_mem_block_head) || memory_size_infimum >= memory_size)
	{
		gmp_mm_block_last_errors = GMP_EC_MM_NOT_ENOUGH_MEM;
		return NULL;
	}

	// preparing area memory head 
	capacity = memory_size / block_size_unit;
	bitmap_size = (capacity + GMP_PORT_DATA_SIZE_PER_BITS - 1) / GMP_PORT_DATA_SIZE_PER_BITS;
	metadata_size = sizeof(gmp_mem_block_head) + offsetof(gmp_mem_area_head, assigned_flag) + bitmap_size;
	used = (metadata_size + block_size_unit - 1) / block_size_unit;
	if (used >= capacity)
	{
		gmp_mm_block_last_errors = GMP_EC_MM_NOT_ENOUGH_MEM;
		return NULL;
	}

	memset(memory_entry, 0, memory_size);

	gmp_mem_block_head* block_head = (gmp_mem_block_head*)memory_entry;
	
	// prepare the first memory block head
	block_head->block_index = 0;
	block_head->block_size = used;
	block_head->magic_number = GMP_MEM_MAGIC_NUMBER;
	block_head->next = NULL;

	// Check if block head has written
	if (*(uint_least16_t*)memory_entry != GMP_MEM_MAGIC_NUMBER)
	{
		gmp_mm_block_last_errors = GMP_EC_MM_WRITE_REFUSE;
		return NULL;
	}


	// construct the memory head
	gmp_mem_area_head* area_head = (gmp_mem_area_head*)((byte_gt*)memory_entry + sizeof(gmp_mem_block_head));

	area_head->entry = memory_entry;
	area_head->block_size_unit = block_size_unit;
	area_head->capacity = capacity;
	area_head->used = used;
	area_head->memory_state = 0; // NOT USE RIGHT NOW.
	area_head->next = NULL;
	area_head->head = block_head;

	// Check if block has written
	if (area_head->memory_state != 0)
	{
		gmp_mm_block_last_errors = GMP_EC_MM_WRITE_REFUSE;
		return NULL;
	}

	// Set assigned_flag for area_head
	set_assigned_flag((gmp_mem_area_head*)area_head, 0, used);

	gmp_mm_block_last_errors = GMP_EC_OK; // Clear flags

	return (gmp_mem_area_head*)area_head;
}


void* gmp_mm_block_alloc(
	gmp_mem_area_head* handle,
size_gt length
) //GMP_NO_OPT
{
	if (handle == NULL || length == 0)
	{
		gmp_mm_block_last_errors = GMP_EC_INVALID_PARAM;
		return NULL;
	}

	// translate length -> block num
	size_gt length_per_unit = (length + sizeof(gmp_mem_block_head) + handle->block_size_unit - 1)
		/ handle->block_size_unit;
	size_gt current_index = 0;
	size_gt current_subindex = 0;

	byte_gt* assigned_flag = &handle->assigned_flag;

	// loop variables
	size_gt i, j;

	for (i = 0; i < handle->capacity; ++i)
	{
		// boundary check
		if (length_per_unit > handle->capacity - i)
		{
			gmp_mm_block_last_errors = GMP_EC_MM_NOT_ENOUGH_MEM;
			return NULL;
		}

		// Check if has a continuous spare space
		for (j = 0; j < length_per_unit; ++j)
		{
			current_index = (i + j) / GMP_PORT_DATA_SIZE_PER_BITS;
			current_subindex = (i + j) % GMP_PORT_DATA_SIZE_PER_BITS;

			if ((assigned_flag[current_index] & (1 << current_subindex)) != 0)
			{
				break;
			}
		}

		// judge whether this space fulfill the condition
		if (j == length_per_unit)
		{
			// clear error flags
			gmp_mm_block_last_errors = GMP_EC_OK;

			handle->used += length_per_unit;


			// fill the block_head struct, and refresh handle
			return fill_block_head(handle, i, length_per_unit);
		}
	}

	gmp_mm_block_last_errors = GMP_EC_MM_NOT_ENOUGH_MEM;
	return NULL;
}


static void clear_assigned_flag(gmp_mem_area_head* handle,
	size_gt position,
	size_gt length
) //GMP_NO_OPT
{
	byte_gt* assigned_flag = &handle->assigned_flag;

	size_gt first_item_index = position / GMP_PORT_DATA_SIZE_PER_BITS;
	size_gt first_item_subindex = position % GMP_PORT_DATA_SIZE_PER_BITS;

	size_gt last_item_index = (position + length) / GMP_PORT_DATA_SIZE_PER_BITS;
	size_gt last_item_subindex = (position + length) % GMP_PORT_DATA_SIZE_PER_BITS;

	size_gt i;

	// Deal with the first item
	for (i = first_item_subindex;
		i < last_item_subindex ||
		((first_item_index < last_item_index) && (i < GMP_PORT_DATA_SIZE_PER_BITS));
		++i)
		assigned_flag[first_item_index] &= ~(1 << i);

	if (first_item_index == last_item_index) // only the first item
		return;

	// Deal with the last item
	for (i = 0; i < last_item_subindex; ++i)
		assigned_flag[last_item_index] &= ~(1 << i);

	if (first_item_index == last_item_index - 1)
		return;

	// Deal with the medium items
	for (i = first_item_index + 1; i < last_item_index; ++i)
		assigned_flag[i] = (byte_gt)0; // set all bits to 0

	return;
}


void gmp_mm_block_free(
	gmp_mem_area_head* handle,
	void* ptr
) //GMP_NO_OPT
{
	if (handle == NULL || ptr == NULL)
	{
		gmp_mm_block_last_errors = GMP_EC_INVALID_PARAM;
		return;
	}

	gmp_mem_block_head* block_head = ((gmp_mem_block_head*)ptr) - 1;
	gmp_mem_block_head* block_head_pos = handle->head;
	gmp_mem_block_head* previous = NULL;
	
	// Check block header format
	if (block_head->magic_number != GMP_MEM_MAGIC_NUMBER)
	{
		gmp_mm_block_last_errors = GMP_EC_INVALID_PARAM;
		return;
	}

	while (block_head_pos != NULL && block_head_pos != block_head)
	{
		previous = block_head_pos;
		block_head_pos = block_head_pos->next;
	}

	// not in the queue
	if (block_head_pos == NULL)
	{
		gmp_mm_block_last_errors = GMP_EC_MM_NO_SPECIFIED_BLOCK;
		return;
	}

	if (previous == NULL)
		handle->head = block_head->next;
	else
		previous->next = block_head->next;

	// Clear magic number
	block_head->magic_number = 0;

	// release the memory allocation
	clear_assigned_flag(handle, block_head->block_index, block_head->block_size);
	handle->used -= block_head->block_size;
	gmp_mm_block_last_errors = GMP_EC_OK;

	return;
}

#endif // SPECIFY_GMP_BLOCK_MEMORY_ENABLE
