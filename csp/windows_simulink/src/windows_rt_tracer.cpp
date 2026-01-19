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

#include <gmp_core.h>

#include <ctrl_rt_trace.h>

#include <io.h>

#define SAVING_CTRL_RT_TRACE_LOG_PATH "rt_trace/crt_bin"
#define CONFIG_RT_TRACE_FILE          "rt_trace/rt_trace_layout.json"

// 1. 初始化 (设定控制周期)
void trace_rt_entity_init(trace_rt_context_t* c, double period_ms)
{
    c->head = NULL;
    c->tail = NULL;
    c->period_ms = period_ms;

    // check if target path exists
    try
    {
        if (std::filesystem::create_directories(SAVING_CTRL_RT_TRACE_LOG_PATH))
        {
            printf("[INFO] Saving directory is creating at: %s.\r\n", SAVING_CTRL_RT_TRACE_LOG_PATH);
        }
        else
        {
            printf("[INFO] Saving directory is : %s.\r\n", SAVING_CTRL_RT_TRACE_LOG_PATH);
        }
    }
    catch (const std::filesystem::filesystem_error& e)
    {
        std::cerr << "[ERROR] Cannot access trace rt directory. Detail: " << e.what();
    }
}

// 2. 注册变量 (返回句柄，以便直接写入，避免字符串查找开销)
trace_rt_node_t* trace_rt_register_node(trace_rt_context_t* c, const char* name, trace_rt_type type)
{
    // create a trace rt node object
    trace_rt_node_t* node = (trace_rt_node_t*)malloc(sizeof(trace_rt_node_t));
    if (!node)
        return NULL;

    memset(node, 0, sizeof(trace_rt_node_t));
    try
    {
        strncpy_s(node->name, name, 63);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ERROR] Wrong node name, " << name << std::endl;
        std::cerr << "Exception detail: " << e.what() << std::endl;
    }

    node->type = type;

    // Get target type
    switch (type)
    {
    case TRT_TYPE_FLOAT:
        node->data_size = sizeof(float);
        break;
    case TRT_TYPE_DOUBLE:
        node->data_size = sizeof(double);
        break;
    case TRT_TYPE_INT32:
        node->data_size = sizeof(int32_t);
        break;
    }

    // generate file name
    try
    {
        sprintf_s(node->filename, "crt_bin/rt_trace_%s.crt_bin", name);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ERROR] Wrong filename, " << name << std::endl;
        std::cerr << "Exception detail: " << e.what() << std::endl;
    }

    // open rt trace file
    try
    {
        fopen_s(&node->fp, node->filename, "wb");

        if (node->fp)
        {
            setvbuf(node->fp, NULL, _IONBF, 1024);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ERROR] Cannot open rt trace data file, " << node->filename << std::endl;
        std::cerr << "Exception detail: " << e.what() << std::endl;
    }

    // 初始化去重计数器 (假设 tick 从 1 开始，0 为初始无效值)
    node->last_tick = 0;

    // 插入链表尾部
    node->next = NULL;
    if (c->tail)
    {
        c->tail->next = node;
        c->tail = node;
    }
    else
    {
        c->head = node;
        c->tail = node;
    }

    return node;
}

// 3. 生成元数据 layout.json
void gmp_trace_rt_generate_layout(trace_rt_context_t* c)
{
    std::fstream config_json(CONFIG_RT_TRACE_FILE, std::fstream::out);

    if (!config_json.is_open())
    {
        std::cout << "[WARN] Cannot open rt trace layout config file." << std::endl;
        return;
    }

    // config json
    nlohmann::json config;

    try
    {
        config["period_ms"] = c->period_ms;

        config["signals"] = nlohmann::json::array();

        trace_rt_node_t* cur = c->head;
        while (cur)
        {
            // 3. 创建一个临时的 json 对象来保存当前节点数据
            nlohmann::json signal_node;

            signal_node["name"] = std::string(cur->name);

            const char* type_str = (cur->type == TRT_TYPE_FLOAT)    ? "float"
                                   : (cur->type == TRT_TYPE_DOUBLE) ? "double"
                                                                    : "int32";
            signal_node["type"] = std::string(type_str);

            signal_node["size"] = cur->data_size;
            signal_node["filename"] = std::string(cur->filename);

            // save a node
            config["signals"].push_back(signal_node);

            // goto next item
            cur = cur->next;
        }

        // save this config file
        config_json << config.dump(4);

        // release file handle
        config_json.close();
    }
    catch (const std::exception& e)
    {
        std::cout << "[ERRO] Cannot generate realtime config json file, " << CONFIG_RT_TRACE_FILE << "." << std::endl;
        std::cerr << e.what() << std::endl;

        return;
    }
}

// 4. 通用写入函数 (核心逻辑：带时间戳 + 去重)
// 参数：handle-变量句柄，tick-当前控制器步数，data-数据指针
void gmp_trace_rt_log_raw(trace_rt_node_t* node, uint32_t tick, void* data)
{
    if (!node || !node->fp)
        return;

    // 【去重逻辑】如果当前时间戳 <= 上次记录的时间戳，说明是同一次调度内的重复输入
    if (tick <= node->last_tick)
    {
        return;
    }

    // 1. 写入时间戳 (4 bytes)
    fwrite(&tick, sizeof(uint32_t), 1, node->fp);

    // 2. 写入数据 (N bytes)
    fwrite(data, node->data_size, 1, node->fp);

    // 3. 刷新缓冲区 (确保 Python 能实时读到)
    fflush(node->fp);

    // 强制提交到磁盘/更新元数据
    _commit(_fileno(node->fp));

    // 更新状态
    node->last_tick = tick;
}

// 辅助封装宏，方便调用
void gmp_trace_rt_log_float(trace_rt_node_t* node, uint32_t tick, float val)
{
    gmp_trace_rt_log_raw(node, tick, &val);
}
void gmp_trace_rt_log_double(trace_rt_node_t* node, uint32_t tick, double val)
{
    gmp_trace_rt_log_raw(node, tick, &val);
}
void gmp_trt_log_int(trace_rt_node_t* node, uint32_t tick, int32_t val)
{
    gmp_trace_rt_log_raw(node, tick, &val);
}

// 5. 清理资源
void gmp_trace_rt_release(trace_rt_context_t* c)
{
    trace_rt_node_t* cur = c->head;
    while (cur)
    {
        trace_rt_node_t* next = cur->next;
        if (cur->fp)
            fclose(cur->fp);
        // release heap
        free(cur);
        cur = next;
    }
    c->head = NULL;
    c->tail = NULL;
}
