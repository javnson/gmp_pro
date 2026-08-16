/** @file sdo_engine.h @brief CANopen expedited and segmented SDO server. */

#ifndef _FILE_GMP_CANOPEN_SDO_ENGINE_H_
#define _FILE_GMP_CANOPEN_SDO_ENGINE_H_

#include <core/protocol/canopen/od.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef GMP_CANOPEN_SDO_MAX_TRANSFER
#define GMP_CANOPEN_SDO_MAX_TRANSFER 128U
#endif

#define GMP_CANOPEN_SDO_ABORT_TOGGLE       0x05030000UL
#define GMP_CANOPEN_SDO_ABORT_COMMAND      0x05040001UL
#define GMP_CANOPEN_SDO_ABORT_OUT_OF_MEMORY 0x05040005UL
#define GMP_CANOPEN_SDO_ABORT_UNSUPPORTED  0x06010000UL
#define GMP_CANOPEN_SDO_ABORT_READ_ONLY    0x06010002UL
#define GMP_CANOPEN_SDO_ABORT_NOT_FOUND    0x06020000UL
#define GMP_CANOPEN_SDO_ABORT_TYPE_LENGTH  0x06070010UL
#define GMP_CANOPEN_SDO_ABORT_GENERAL      0x08000000UL

typedef enum
{
    GMP_CANOPEN_SDO_IDLE = 0,
    GMP_CANOPEN_SDO_SEGMENTED_DOWNLOAD,
    GMP_CANOPEN_SDO_SEGMENTED_UPLOAD
} gmp_canopen_sdo_state_t;

typedef struct
{
    uint16_t node_id;
    gmp_canopen_od_t* dictionary;
    gmp_canopen_sdo_state_t state;
    gmp_canopen_od_entry_t* entry;
    uint32_t offset;
    uint32_t total_size;
    uint16_t toggle;
    gmp_canopen_octet_t transfer[GMP_CANOPEN_SDO_MAX_TRANSFER];
} gmp_canopen_sdo_server_t;

fast_gt gmp_canopen_sdo_server_init(gmp_canopen_sdo_server_t* server,
                                     uint16_t node_id,
                                     gmp_canopen_od_t* dictionary);
void gmp_canopen_sdo_server_reset(gmp_canopen_sdo_server_t* server);
fast_gt gmp_canopen_sdo_server_process(gmp_canopen_sdo_server_t* server,
                                       const gmp_canopen_frame_t* request,
                                       gmp_canopen_frame_t* response);
fast_gt gmp_canopen_sdo_server_receive(gmp_canopen_sdo_server_t* server,
                                       const gmp_canopen_frame_t* request,
                                       gmp_canopen_send_fn send,
                                       void* send_context);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_SDO_ENGINE_H_ */
