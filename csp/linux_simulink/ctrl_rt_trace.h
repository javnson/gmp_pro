/**
 * @file ctrl_rt_trace.h
 * @brief Portable binary signal tracing for hosted GMP controllers.
 */

#ifndef GMP_LINUX_SIMULINK_CTRL_RT_TRACE_H
#define GMP_LINUX_SIMULINK_CTRL_RT_TRACE_H

#include <stdint.h>
#include <stdio.h>

typedef enum
{
    TRT_TYPE_FLOAT,
    TRT_TYPE_DOUBLE,
    TRT_TYPE_INT32
} trace_rt_type;

struct gmp_trace_rt_context;

typedef struct gmp_trace_rt_node
{
    char name[64];
    trace_rt_type type;
    int data_size;
    FILE* fp;
    char filename[256];
    uint32_t last_tick;
    uint8_t has_sample;
    struct gmp_trace_rt_context* parent;
    struct gmp_trace_rt_node* next;
} trace_rt_node_t;

typedef struct gmp_trace_rt_context
{
    trace_rt_node_t* head;
    trace_rt_node_t* tail;
    double period_ms;
} trace_rt_context_t;

/** @brief Initialize a trace context and its output directory. */
void trace_rt_entity_init(trace_rt_context_t* context, double period_ms);

/** @brief Register one scalar trace signal. */
trace_rt_node_t* trace_rt_register_node(trace_rt_context_t* context, const char* name, trace_rt_type type);

/** @brief Write the trace metadata document. */
void gmp_trace_rt_generate_layout(trace_rt_context_t* context);

/** @brief Append one raw scalar sample when its tick is newer. */
void gmp_trace_rt_log_raw(trace_rt_node_t* node, uint32_t tick, const void* data);

/** @brief Flush and release all signals owned by a trace context. */
void gmp_trace_rt_release(trace_rt_context_t* context);

/** @brief Append one single-precision sample. */
void gmp_trace_rt_log_float(trace_rt_node_t* node, uint32_t tick, float value);

/** @brief Append one double-precision sample. */
void gmp_trace_rt_log_double(trace_rt_node_t* node, uint32_t tick, double value);

/** @brief Append one signed 32-bit integer sample. */
void gmp_trace_rt_log_int(trace_rt_node_t* node, uint32_t tick, int32_t value);

#endif /* GMP_LINUX_SIMULINK_CTRL_RT_TRACE_H */
