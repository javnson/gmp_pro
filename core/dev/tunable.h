/**
 * @file gmp_param.h
 * @brief Tunable Parameter Group Management (Dictionary-based Data Access)
 * @details Provides a safe, whitelist-based mechanism for dynamically reading and 
 * writing discrete variables over the datalink using a static data dictionary.
 */

#include <core/dev/datalink.h>

#ifndef GMP_PARAM_H
#define GMP_PARAM_H

/**
 * @brief Variable type definitions.
 * @details Determines the payload size and formatting of the variable on the bus.
 */
typedef enum
{
    GMP_PARAM_TYPE_U16 = 0, /**< Unsigned 16-bit integer */
    GMP_PARAM_TYPE_I16,     /**< Signed 16-bit integer */
    GMP_PARAM_TYPE_U32,     /**< Unsigned 32-bit integer */
    GMP_PARAM_TYPE_I32,     /**< Signed 32-bit integer */
    GMP_PARAM_TYPE_F32      /**< 32-bit single-precision floating point */
} gmp_param_type_t;

// =========================================================
// Read/Write Permission Attributes
// =========================================================

/** @brief Read-only parameter permission */
#define GMP_PARAM_PERM_RO 0x00
/** @brief Read-write parameter permission */
#define GMP_PARAM_PERM_RW 0x01

/**
 * @brief Data dictionary entry structure.
 * @details Represents a single tunable parameter. Typically stored in static memory.
 */
typedef struct
{
    void* addr;            /**< Physical memory address of the variable */
    gmp_param_type_t type; /**< Data type of the variable */
    fast16_gt perm;        /**< Permission attribute (DSP 16-bit alignment compatible) */
} gmp_param_item_t;

/**
 * @brief Tunable parameter group object (Class Context).
 * @details Manages the datalink binding and the static dictionary mapping.
 */
typedef struct
{
    gmp_datalink_t* dl_ctx;       /**< Bound datalink communication object */
    uint16_t base_cmd;            /**< Base command ID occupied (Read = base_cmd, Write = base_cmd + 1) */
    const gmp_param_item_t* dict; /**< Pointer to the bound data dictionary array */
    fast16_gt dict_size;          /**< Maximum capacity of the dictionary (Max supported: 255) */
} gmp_param_tunable_t;

// =========================================================
// API Declarations
// =========================================================

/**
 * @brief Initialize the tunable parameter group object.
 * @param ctx Pointer to the tunable parameter context to initialize.
 * @param dl Pointer to the bound datalink communication object.
 * @param base_cmd The base command ID to be occupied by this service.
 * @param dict Pointer to the static data dictionary array.
 * @param dict_size Number of items in the data dictionary array.
 */
void gmp_param_tunable_init(gmp_param_tunable_t* ctx, gmp_datalink_t* dl, uint16_t base_cmd,
                            const gmp_param_item_t* dict, fast16_gt dict_size);

/**
 * @brief Tunable parameter group reception callback.
 * @details Must be placed within the datalink RX_OK event handling chain.
 * @param ctx Pointer to the tunable parameter context.
 * @return fast_gt Returns 1 if the command belongs to this object and is handled, 0 otherwise.
 */
fast_gt gmp_param_tunable_rx_cb(gmp_param_tunable_t* ctx);

#endif // GMP_PARAM_H
