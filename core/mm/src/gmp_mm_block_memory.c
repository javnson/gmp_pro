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

#include <gmp_core.h>

#if defined SPECIFY_GMP_BLOCK_MEMORY_ENABLE

//global variables
gmp_stat_t gmp_mem_block_last_errors = GMP_STAT_OK;


// utilities

static void set_assigned_flag(gmp_mem_area_head* handle,
	size_gt position,
	size_gt length
) //GMP_NO_OPT
{
	data_gt* assigned_flag = &handle->assigned_flag;

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
		assigned_flag[i] = (data_gt)-1; // set all bits to 1

	return;
}

static void* fill_block_head(gmp_mem_area_head* handle,
	size_gt position,
	size_gt length
) //GMP_NO_OPT
{
	gmp_mem_block_head* block_head = (gmp_mem_block_head*)
		(((data_gt*)handle->entry) + position * handle->block_size_unit);

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
gmp_mem_area_head* gmp_mem_setup(	// return the memory area handle
	void* memory_entry,				// entry of the memory block
	uint32_t memory_size,		    // bytes
	size_gt block_size_unit
) //GMP_NO_OPT
{
	uint32_t memory_size_infimum = sizeof(gmp_mem_area_head) + sizeof(gmp_mem_block_head);

	// not enough memory
	if (memory_size_infimum >= memory_size)
	{
		gmp_mem_block_last_errors = GMP_STAT_MM_NOT_ENOUGH_MEM;
		return NULL;
	}

	// preparing area memory head 
	size_gt capacity = memory_size / block_size_unit;
	size_gt used = (sizeof(gmp_mem_block_head)
		+ sizeof(gmp_mem_area_head)
		+ capacity / sizeof(data_gt) / GMP_PORT_DATA_SIZE_PER_BITS)
		/ block_size_unit + 1;

	gmp_mem_block_head* block_head = (gmp_mem_block_head*)memory_entry;
	
	// prepare the first memory block head
	block_head->block_index = 0;
	block_head->block_size = used;
	block_head->magic_number = GMP_MEM_MAGIC_NUMBER;
	block_head->next = NULL;

	// Check if block head has written
	if (*(uint_least16_t*)memory_entry != GMP_MEM_MAGIC_NUMBER)
	{
		gmp_mem_block_last_errors = GMP_STAT_MM_WRITE_REFUSE;
		return NULL;
	}


	// construct the memory head
	gmp_mem_area_head* area_head = (gmp_mem_area_head*)((data_gt*)memory_entry + sizeof(gmp_mem_block_head));
	data_gt* assigned_flag = &area_head->assigned_flag;

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
		gmp_mem_block_last_errors = GMP_STAT_MM_WRITE_REFUSE;
		return NULL;
	}

	// Set assigned_flag for area_head
	set_assigned_flag((gmp_mem_area_head*)area_head, 0, used);

	gmp_mem_block_last_errors = GMP_STAT_OK; // Clear flags

	return (gmp_mem_area_head*)area_head;
}


void* gmp_block_alloc(
	gmp_mem_area_head* handle,
size_gt length
) //GMP_NO_OPT
{
	// translate length -> block num
	size_gt length_per_unit = (length + sizeof(gmp_mem_block_head))
		/ handle->block_size_unit + 1;
	size_gt current_index = 0;
	size_gt current_subindex = 0;

	data_gt* assigned_flag = &handle->assigned_flag;

	// loop variables
	size_gt i, j;

	for (i = 0; i < handle->capacity; ++i)
	{
		// boundary check
		if (length_per_unit > handle->capacity - i)
		{
			gmp_mem_block_last_errors = GMP_STAT_MM_NOT_ENOUGH_MEM;
			return NULL;
		}

		// Check if has a continuous spare space
		for (j = 0; j < length_per_unit; ++j)
		{
			current_index = (i + j) / GMP_PORT_DATA_SIZE_PER_BITS;
			current_subindex = (i + j) % GMP_PORT_DATA_SIZE_PER_BITS;

			if ((assigned_flag[current_index] & (1 << current_subindex)) != NULL)
			{
				break;
			}
		}

		// judge whether this space fulfill the condition
		if (j == length_per_unit)
		{
			// clear error flags
			gmp_mem_block_last_errors = GMP_STAT_OK;

			handle->used += length_per_unit;


			// fill the block_head struct, and refresh handle
			return fill_block_head(handle, i, length_per_unit);
		}
	}

	gmp_mem_block_last_errors = GMP_STAT_MM_NOT_ENOUGH_MEM;
	return NULL;
}


static void clear_assigned_flag(gmp_mem_area_head* handle,
	size_gt position,
	size_gt length
) //GMP_NO_OPT
{
	data_gt* assigned_flag = &handle->assigned_flag;

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
		assigned_flag[i] = (data_gt)0; // set all bits to 0

	return;
}


void gmp_block_free(
	gmp_mem_area_head* handle,
	void* ptr
) //GMP_NO_OPT
{
	gmp_mem_block_head* block_head = ((gmp_mem_block_head*)ptr) - 1;
	gmp_mem_block_head* block_head_pos = handle->head;

	size_gt cnt;
	
	// Check block header format
	if (block_head->magic_number != GMP_MEM_MAGIC_NUMBER)
	{
		gmp_mem_block_last_errors = GMP_STAT_INVALID_PARAM;
		return;
	}

	// initialize a counter avoiding endless loop
	cnt = 0;
	// look for the previous node
	// handle the first item
	if (block_head_pos == block_head)
	{
		handle->head = block_head->next;
	}
	else
		while (block_head_pos != NULL)
		{
			if (cnt > handle->used)
			{
				// not in the queue
				gmp_mem_block_last_errors = GMP_STAT_MM_NO_SPECIFIED_BLOCK;
			}

			// look for the position to be delete
			if (block_head_pos->next == block_head)
			{
				// remove the node
				block_head_pos->next = block_head->next;
				break;
			}

			block_head_pos += 1;
		}

	// not in the queue
	if (block_head_pos == NULL)
	{
		gmp_mem_block_last_errors = GMP_STAT_MM_NO_SPECIFIED_BLOCK;
		return;
	}

	// Clear magic number
	block_head->magic_number = 0;

	// release the memory allocation
	clear_assigned_flag(handle, block_head->block_index, block_head->block_size);

	return;
}

#endif // SPECIFY_GMP_BLOCK_MEMORY_ENABLE
