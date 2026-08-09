/**
 * @file gmp_scope.c
 * @brief Data Link Scope service with target-controlled capture and sample reads.
 */

#include <gmp_core.h>
#include <core/dev/scope.h>

#include <stdint.h>
#include <string.h>

/** @brief Read one protocol byte and advance the payload cursor. */
static fast_gt scope_read_u8(const data_gt* payload, uint16_t length,
                             uint16_t* index, fast16_gt* value)
{
    if (*index >= length)
        return 0;
    *value = payload[*index] & 0xFF;
    *index = (uint16_t)(*index + 1U);
    return 1;
}

/** @brief Read one little-endian unsigned 16-bit value. */
static fast_gt scope_read_u16(const data_gt* payload, uint16_t length,
                              uint16_t* index, uint16_t* value)
{
    fast16_gt low;
    fast16_gt high;
    if (!scope_read_u8(payload, length, index, &low) ||
        !scope_read_u8(payload, length, index, &high))
        return 0;
    *value = (uint16_t)low | (uint16_t)((uint16_t)high << 8);
    return 1;
}

/** @brief Read one little-endian unsigned 32-bit value. */
static fast_gt scope_read_u32(const data_gt* payload, uint16_t length,
                              uint16_t* index, uint32_t* value)
{
    fast16_gt byte_0;
    fast16_gt byte_1;
    fast16_gt byte_2;
    fast16_gt byte_3;
    if (!scope_read_u8(payload, length, index, &byte_0) ||
        !scope_read_u8(payload, length, index, &byte_1) ||
        !scope_read_u8(payload, length, index, &byte_2) ||
        !scope_read_u8(payload, length, index, &byte_3))
        return 0;
    *value = (uint32_t)byte_0 | ((uint32_t)byte_1 << 8) |
             ((uint32_t)byte_2 << 16) | ((uint32_t)byte_3 << 24);
    return 1;
}

/** @brief Begin a Scope response and append its operation and status fields. */
static void scope_begin_response(gmp_scope_service_t* ctx, fast16_gt operation,
                                 fast16_gt status)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, dl->rx_head.cmd);
    gmp_dev_dl_tx_append_u8(dl, (data_gt)operation);
    gmp_dev_dl_tx_append_u8(dl, (data_gt)status);
}

/** @brief Complete the current Scope response and mark its request handled. */
static void scope_finish_response(gmp_scope_service_t* ctx)
{
    gmp_dev_dl_tx_ready(ctx->dl_ctx);
    gmp_dev_dl_msg_handled(ctx->dl_ctx);
}

/** @brief Return one byte from a native scope buffer on either platform family. */
static data_gt scope_buffer_byte(const gmp_scope_resource_t* resource,
                                 uint32_t byte_offset)
{
#if GMP_PORT_DATA_SIZE_PER_BYTES == 1
    const data_gt* buffer = (const data_gt*)resource->buffer;
    return buffer[byte_offset];
#elif GMP_PORT_DATA_SIZE_PER_BYTES == 2
    const data_gt* buffer = (const data_gt*)resource->buffer;
    data_gt native_unit = buffer[byte_offset >> 1];
    return (byte_offset & 1U) ? (data_gt)((native_unit >> 8) & 0xFF) :
                               (data_gt)(native_unit & 0xFF);
#else
#error "Unsupported GMP Data Link Scope data unit"
#endif
}

/** @brief Reply with one indexed scope resource descriptor. */
static void scope_reply_discovery(gmp_scope_service_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    fast16_gt resource_id = 0;
    fast16_gt total = ctx->resource_count;
    const gmp_scope_resource_t* resource = NULL;
    const char* name = NULL;
    size_gt name_length = 0U;
    size_gt index;
    size_gt capacity;

    if (dl->expected_payload_len >= 2U)
        resource_id = dl->payload_buf[1] & 0xFF;
    if (resource_id < total)
        resource = &ctx->resources[resource_id];

    scope_begin_response(ctx, GMP_SCOPE_OP_DISCOVER,
                         (resource == NULL) ? 1 : 0);
    gmp_dev_dl_tx_append_u8(dl, GMP_SCOPE_PROTOCOL_VERSION);
    gmp_dev_dl_tx_append_u8(dl, (data_gt)total);
    gmp_dev_dl_tx_append_u8(dl, (data_gt)resource_id);
    if (resource != NULL)
    {
        gmp_dev_dl_tx_append_u8(dl, (data_gt)resource->sample_type);
        gmp_dev_dl_tx_append_u8(dl, (data_gt)resource->layout);
        gmp_dev_dl_tx_append_u16(dl, resource->channels);
        gmp_dev_dl_tx_append_u32(dl, resource->depth);
        gmp_dev_dl_tx_append_u32(dl, resource->sample_rate_hz);
        gmp_dev_dl_tx_append_u32(dl, resource->byte_length);
        name = resource->name;
        capacity = gmp_dev_dl_get_tx_capacity(dl);
        if (name != NULL && capacity > 1U)
        {
            name_length = strlen(name);
            if (name_length > capacity - 1U)
                name_length = capacity - 1U;
            if (name_length > 63U)
                name_length = 63U;
        }
        gmp_dev_dl_tx_append_u8(dl, (data_gt)name_length);
        for (index = 0U; index < name_length; ++index)
            gmp_dev_dl_tx_append_u8(dl, (data_gt)name[index]);
    }
    scope_finish_response(ctx);
}

/** @brief Decode and apply a scope configuration request. */
static void scope_reply_configure(gmp_scope_service_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    const data_gt* payload = dl->payload_buf;
    uint16_t index = 1U;
    fast16_gt resource_id = 0;
    fast16_gt mode = 0;
    fast16_gt channel = 0;
    uint16_t position = 0U;
    uint32_t level_bits = 0U;
    uint32_t timeout_ms = 0U;
    uint16_t sample_divider = 0U;
    float level_f32 = 0.0F;
    fast16_gt status = 0;
    gmp_scope_config_t config;
    const gmp_scope_resource_t* resource = NULL;

    if ((dl->expected_payload_len != 14U && dl->expected_payload_len != 16U) ||
        !scope_read_u8(payload, dl->expected_payload_len, &index, &resource_id) ||
        !scope_read_u8(payload, dl->expected_payload_len, &index, &mode) ||
        !scope_read_u8(payload, dl->expected_payload_len, &index, &channel) ||
        !scope_read_u16(payload, dl->expected_payload_len, &index, &position) ||
        !scope_read_u32(payload, dl->expected_payload_len, &index, &level_bits) ||
        !scope_read_u32(payload, dl->expected_payload_len, &index, &timeout_ms))
        status = 1;
    if (status == 0 && dl->expected_payload_len == 16U &&
        !scope_read_u16(payload, dl->expected_payload_len, &index, &sample_divider))
        status = 1;
    if (status == 0 && resource_id < ctx->resource_count)
        resource = &ctx->resources[resource_id];
    if (resource == NULL || resource->configure == NULL)
        status = 2;
    if (status == 0)
    {
        memcpy(&level_f32, &level_bits, sizeof(level_f32));
        config.mode = mode;
        config.channel = channel;
        config.position_permille = position;
        config.level = (parameter_gt)level_f32;
        config.auto_timeout_ms = timeout_ms;
        config.sample_divider = sample_divider;
        if (!resource->configure(resource->user_context, &config))
            status = 3;
    }
    scope_begin_response(ctx, GMP_SCOPE_OP_CONFIGURE, status);
    gmp_dev_dl_tx_append_u8(dl, (data_gt)resource_id);
    scope_finish_response(ctx);
}

/** @brief Arm one registered scope resource. */
static void scope_reply_arm(gmp_scope_service_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    fast16_gt resource_id = (dl->expected_payload_len >= 2U) ?
                                (dl->payload_buf[1] & 0xFF) : 255;
    fast16_gt status = 0;
    if (dl->expected_payload_len != 2U || resource_id >= ctx->resource_count ||
        ctx->resources[resource_id].arm == NULL)
        status = 1;
    else if (!ctx->resources[resource_id].arm(
                 ctx->resources[resource_id].user_context))
        status = 2;
    scope_begin_response(ctx, GMP_SCOPE_OP_ARM, status);
    gmp_dev_dl_tx_append_u8(dl, (data_gt)resource_id);
    scope_finish_response(ctx);
}

/** @brief Return state and generation for one registered scope resource. */
static void scope_reply_status(gmp_scope_service_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    fast16_gt resource_id = (dl->expected_payload_len >= 2U) ?
                                (dl->payload_buf[1] & 0xFF) : 255;
    fast16_gt status = 0;
    uint32_t generation = 0U;
    gmp_scope_capture_state_t state = GMP_SCOPE_STATE_WAITING;
    if (dl->expected_payload_len != 2U || resource_id >= ctx->resource_count ||
        ctx->resources[resource_id].get_status == NULL)
        status = 1;
    else
        state = ctx->resources[resource_id].get_status(
            ctx->resources[resource_id].user_context, &generation);
    scope_begin_response(ctx, GMP_SCOPE_OP_STATUS, status);
    gmp_dev_dl_tx_append_u8(dl, (data_gt)resource_id);
    gmp_dev_dl_tx_append_u8(dl, (data_gt)state);
    gmp_dev_dl_tx_append_u32(dl, generation);
    scope_finish_response(ctx);
}

/** @brief Return one bounded chunk from a ready scope buffer. */
static void scope_reply_read(gmp_scope_service_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    const data_gt* payload = dl->payload_buf;
    uint16_t index = 1U;
    fast16_gt resource_id = 255;
    uint32_t offset = 0U;
    uint16_t length = 0U;
    fast16_gt status = 0;
    const gmp_scope_resource_t* resource = NULL;
    size_gt byte_index;

    if (dl->expected_payload_len != 8U ||
        !scope_read_u8(payload, dl->expected_payload_len, &index, &resource_id) ||
        !scope_read_u32(payload, dl->expected_payload_len, &index, &offset) ||
        !scope_read_u16(payload, dl->expected_payload_len, &index, &length))
        status = 1;
    if (status == 0 && resource_id < ctx->resource_count)
        resource = &ctx->resources[resource_id];
    if (resource == NULL || resource->buffer == NULL || length == 0U ||
        offset > resource->byte_length || length > resource->byte_length - offset)
        status = 2;
    if (status == 0 &&
        (resource->get_status == NULL ||
         resource->get_status(resource->user_context, NULL) != GMP_SCOPE_STATE_READY))
    {
        /* A readable snapshot owns the buffer until a later ARM request. */
        status = 4;
    }

    scope_begin_response(ctx, GMP_SCOPE_OP_READ, status);
    gmp_dev_dl_tx_append_u8(dl, (data_gt)resource_id);
    gmp_dev_dl_tx_append_u32(dl, offset);
    if (status == 0 && gmp_dev_dl_get_tx_capacity(dl) < (size_gt)length + 2U)
        status = 3;
    if (status != 0)
    {
        /* Update the already-emitted status field before sealing the response. */
        dl->tx_buf[1] = (data_gt)status;
        length = 0U;
    }
    gmp_dev_dl_tx_append_u16(dl, length);
    for (byte_index = 0U; byte_index < length; ++byte_index)
        gmp_dev_dl_tx_append_u8(dl, scope_buffer_byte(resource, offset + byte_index));
    scope_finish_response(ctx);
}

void gmp_scope_init(gmp_scope_service_t* ctx, gmp_datalink_t* dl,
                    uint16_t base_cmd, const gmp_scope_resource_t* resources,
                    fast16_gt resource_count)
{
    if (ctx == NULL)
        return;
    ctx->dl_ctx = dl;
    ctx->base_cmd = base_cmd;
    ctx->resources = resources;
    ctx->resource_count = (resource_count > 255) ? 255 : resource_count;
}

fast_gt gmp_scope_rx_cb(gmp_scope_service_t* ctx)
{
    gmp_datalink_t* dl;
    fast16_gt operation;
    if (ctx == NULL || ctx->dl_ctx == NULL)
        return 0;
    dl = ctx->dl_ctx;
    if (dl->rx_head.cmd != ctx->base_cmd)
        return 0;
    if (dl->expected_payload_len < 1U)
    {
        gmp_dev_dl_reply_nack(dl, 0x0002U);
        gmp_dev_dl_msg_handled(dl);
        return 1;
    }
    operation = dl->payload_buf[0] & 0xFF;
    switch (operation)
    {
    case GMP_SCOPE_OP_DISCOVER:
        scope_reply_discovery(ctx);
        break;
    case GMP_SCOPE_OP_CONFIGURE:
        scope_reply_configure(ctx);
        break;
    case GMP_SCOPE_OP_ARM:
        scope_reply_arm(ctx);
        break;
    case GMP_SCOPE_OP_STATUS:
        scope_reply_status(ctx);
        break;
    case GMP_SCOPE_OP_READ:
        scope_reply_read(ctx);
        break;
    default:
        scope_begin_response(ctx, operation, 0xFF);
        scope_finish_response(ctx);
        break;
    }
    return 1;
}
