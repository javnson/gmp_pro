/**
 * @file windows_rt_tracer.cpp
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

#include <filesystem>
#include <exception>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef _FILE_GMP_CTRL_RT_TRACE_H_
#define _FILE_GMP_CTRL_RT_TRACE_H_

typedef enum
{
    TRT_TYPE_FLOAT,
    TRT_TYPE_DOUBLE,
    TRT_TYPE_INT32
} trace_rt_type;

struct _tag_trace_context;

// trace rt node
typedef struct _tag_trt_node
{
    // monitor target name
    char name[64];

    // data type and data size
    trace_rt_type type;
    int data_size;

    // file handle and file name
    FILE* fp;
    char filename[256];

    // record last tick for deduplication
    uint32_t last_tick;

    // point to trace context
    struct _tag_trace_context* parent;
    
    // point to next object
    struct _tag_trt_node* next;
} trace_rt_node_t;

// global context
typedef struct _tag_trace_context
{
    trace_rt_node_t* head;
    trace_rt_node_t* tail;

    // controller period, unit ms
    double period_ms;
} trace_rt_context_t;

//static TraceContext g_ctx = {NULL, NULL, 20.0}; // 默认 20ms

// ================== 接口实现 ==================
void trace_rt_entity_init(trace_rt_context_t* c, double period_ms);

// 2. 注册变量 (返回句柄，以便直接写入，避免字符串查找开销)
trace_rt_node_t* trace_rt_register_node(trace_rt_context_t* c, const char* name, trace_rt_type type);

// 3. 生成元数据 layout.json
void gmp_trace_rt_generate_layout(trace_rt_context_t* c);

// 4. 通用写入函数 (核心逻辑：带时间戳 + 去重)
void gmp_trace_rt_log_raw(trace_rt_node_t* node, uint32_t tick, void* data);

// 5. 清理资源
void gmp_trace_rt_release(trace_rt_context_t* c);

void gmp_trace_rt_log_float(trace_rt_node_t* node, uint32_t tick, float val);
void gmp_trace_rt_log_double(trace_rt_node_t* node, uint32_t tick, double val);
void gmp_trace_rt_log_int(trace_rt_node_t* node, uint32_t tick, int32_t val);

#endif // _FILE_GMP_CTRL_RT_TRACE_H_
