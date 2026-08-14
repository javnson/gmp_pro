/**
 * @file linux_rt_tracer.cpp
 * @brief Portable realtime trace writer for hosted Linux controllers.
 */

#include <ctrl_rt_trace.h>

#include <nlohmann/json.hpp>

#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>

namespace
{
constexpr const char* trace_root = "rt_trace";
constexpr const char* trace_data_directory = "rt_trace/crt_bin";
constexpr const char* trace_layout_path = "rt_trace/rt_trace_layout.json";

std::string safe_file_component(const char* name)
{
    std::string result;
    for (const unsigned char character : std::string(name))
    {
        if (std::isalnum(character) != 0 || character == '_' || character == '-' || character == '.')
            result.push_back(static_cast<char>(character));
        else
            result.push_back('_');
    }
    return result.empty() ? std::string("signal") : result;
}

int trace_data_size(trace_rt_type type)
{
    switch (type)
    {
    case TRT_TYPE_FLOAT:
        return static_cast<int>(sizeof(float));
    case TRT_TYPE_DOUBLE:
        return static_cast<int>(sizeof(double));
    case TRT_TYPE_INT32:
        return static_cast<int>(sizeof(int32_t));
    default:
        return 0;
    }
}

bool create_directory(const char* path)
{
    return ::mkdir(path, 0755) == 0 || errno == EEXIST;
}
} // namespace

void trace_rt_entity_init(trace_rt_context_t* context, double period_ms)
{
    if (context == nullptr)
        return;

    context->head = nullptr;
    context->tail = nullptr;
    context->period_ms = period_ms;

    if (!create_directory(trace_root) || !create_directory(trace_data_directory))
        std::cerr << "[ERROR] Cannot create the realtime trace directory: " << std::strerror(errno) << std::endl;
    else
        std::cout << "[INFO] Realtime trace directory: " << trace_data_directory << std::endl;
}

trace_rt_node_t* trace_rt_register_node(trace_rt_context_t* context, const char* name, trace_rt_type type)
{
    if (context == nullptr || name == nullptr)
        return nullptr;

    const int data_size = trace_data_size(type);
    if (data_size == 0)
        return nullptr;

    trace_rt_node_t* node = static_cast<trace_rt_node_t*>(std::calloc(1, sizeof(trace_rt_node_t)));
    if (node == nullptr)
        return nullptr;

    std::snprintf(node->name, sizeof(node->name), "%s", name);
    node->type = type;
    node->data_size = data_size;
    node->parent = context;

    const std::string safe_name = safe_file_component(name);
    std::snprintf(node->filename, sizeof(node->filename), "crt_bin/rt_trace_%s.crt_bin", safe_name.c_str());
    char output_path[sizeof(node->filename) + 32];
    std::snprintf(output_path, sizeof(output_path), "%s/%s", trace_root, node->filename);
    node->fp = std::fopen(output_path, "wb");
    if (node->fp == nullptr)
    {
        std::cerr << "[ERROR] Cannot open realtime trace file: " << output_path << std::endl;
        std::free(node);
        return nullptr;
    }

    std::setvbuf(node->fp, nullptr, _IONBF, 0);
    if (context->tail != nullptr)
    {
        context->tail->next = node;
        context->tail = node;
    }
    else
    {
        context->head = node;
        context->tail = node;
    }
    return node;
}

void gmp_trace_rt_generate_layout(trace_rt_context_t* context)
{
    if (context == nullptr)
        return;

    std::ofstream output(trace_layout_path);
    if (!output.is_open())
    {
        std::cerr << "[WARN] Cannot open realtime trace layout file: " << trace_layout_path << std::endl;
        return;
    }

    nlohmann::json layout;
    layout["period_ms"] = context->period_ms;
    layout["signals"] = nlohmann::json::array();
    for (trace_rt_node_t* node = context->head; node != nullptr; node = node->next)
    {
        const char* type_name = node->type == TRT_TYPE_FLOAT    ? "float"
                                : node->type == TRT_TYPE_DOUBLE ? "double"
                                                                : "int32";
        layout["signals"].push_back({{"name", node->name},
                                      {"type", type_name},
                                      {"size", node->data_size},
                                      {"filename", node->filename}});
    }
    output << layout.dump(4) << '\n';
}

void gmp_trace_rt_log_raw(trace_rt_node_t* node, uint32_t tick, const void* data)
{
    if (node == nullptr || node->fp == nullptr || data == nullptr)
        return;
    if (node->has_sample != 0U && tick <= node->last_tick)
        return;

    if (std::fwrite(&tick, sizeof(tick), 1, node->fp) != 1 ||
        std::fwrite(data, static_cast<size_t>(node->data_size), 1, node->fp) != 1)
    {
        std::cerr << "[ERROR] Cannot append realtime trace sample: " << node->filename << std::endl;
        return;
    }
    std::fflush(node->fp);
    node->last_tick = tick;
    node->has_sample = 1U;
}

void gmp_trace_rt_log_float(trace_rt_node_t* node, uint32_t tick, float value)
{
    gmp_trace_rt_log_raw(node, tick, &value);
}

void gmp_trace_rt_log_double(trace_rt_node_t* node, uint32_t tick, double value)
{
    gmp_trace_rt_log_raw(node, tick, &value);
}

void gmp_trace_rt_log_int(trace_rt_node_t* node, uint32_t tick, int32_t value)
{
    gmp_trace_rt_log_raw(node, tick, &value);
}

void gmp_trace_rt_release(trace_rt_context_t* context)
{
    if (context == nullptr)
        return;

    trace_rt_node_t* node = context->head;
    while (node != nullptr)
    {
        trace_rt_node_t* next = node->next;
        if (node->fp != nullptr)
            std::fclose(node->fp);
        std::free(node);
        node = next;
    }
    context->head = nullptr;
    context->tail = nullptr;
}
