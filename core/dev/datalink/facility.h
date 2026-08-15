/**
 * @file facility.h
 * @brief Intrusive service descriptor used by the GMP Data Link dispatcher.
 */

#ifndef _FILE_GMP_DATALINK_FACILITY_H_
#define _FILE_GMP_DATALINK_FACILITY_H_

#include <gmp_type.h>
#include <core/base/ds/list.h>

#ifdef __cplusplus
extern "C"
{
#endif

struct _tag_gmp_datalink;
struct _tag_gmp_dl_facility;

typedef struct _tag_gmp_datalink gmp_datalink_t;
typedef struct _tag_gmp_dl_facility gmp_dl_facility_t;

/** @brief Stable service kinds reported by the Data Link INFO command. */
typedef enum
{
    GMP_DL_FACILITY_INVALID = 0,
    GMP_DL_FACILITY_TUNABLE = 1,
    GMP_DL_FACILITY_MEMORY = 2,
    GMP_DL_FACILITY_SCOPE = 3,
    GMP_DL_FACILITY_PIL = 4,
    GMP_DL_FACILITY_USER = 0x80
} gmp_dl_facility_type_t;

/** @brief Dispatch one received request to a registered service object. */
typedef fast_gt (*gmp_dl_facility_rx_cb_t)(gmp_dl_facility_t* facility,
                                           gmp_datalink_t* datalink);

/**
 * @brief Common header embedded as the first member of every DL submodule.
 *
 * The intrusive list is deliberately the first field and the object type is
 * the second field. This makes list diagnostics and debugger inspection
 * consistent without dynamic allocation or a separate registry object.
 */
struct _tag_gmp_dl_facility
{
    gmp_list list;                    /**< First field: intrusive service link. */
    gmp_dl_facility_type_t type;      /**< Second field: service object type. */
    uint16_t base_cmd;                /**< First command claimed by this service. */
    uint16_t command_count;           /**< Number of consecutive commands claimed. */
    gmp_dl_facility_rx_cb_t dispatch; /**< Type-specific receive dispatcher. */
    void* owner;                      /**< Owning module context. */
};

/** @brief Initialize a detached facility descriptor. */
void gmp_dl_facility_init(gmp_dl_facility_t* facility,
                          gmp_dl_facility_type_t type,
                          uint16_t base_cmd, uint16_t command_count,
                          gmp_dl_facility_rx_cb_t dispatch, void* owner);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_DATALINK_FACILITY_H_ */
