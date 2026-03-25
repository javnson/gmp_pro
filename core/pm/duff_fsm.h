
// Construct a non-blocking task

#ifndef _FILE_GMP_PM_DUFF_FSM_H_
#define _FILE_GMP_PM_DUFF_FSM_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/* ========================================================================== */
/* 数据类型定义                                                               */
/* ========================================================================== */

/**
 * @brief 状态机初始化校验的“魔法字”
 */
#define FSM_MAGIC_WORD (0x5AA5)

/**
 * @brief fsm_wait 宏使用的行号偏移量，防止与用户自定义的 case 编号冲突
 */
#ifndef FSM_LINE_OFFSET
#define FSM_LINE_OFFSET (10007)
#endif

/**
 * @brief 状态机执行结果返回值
 */
typedef enum
{
    FSM_RET_YIELD = 0, /**< 挂起：状态机正在延时或等待，主动让出 CPU */
    FSM_RET_DONE = 1,  /**< 完成：状态机本次生命周期正常结束 */
    FSM_RET_ERROR = 2  /**< 错误：状态机发生异常（如状态跑飞） */
} fsm_ret_t;

/**
 * @brief 状态机上下文控制块 (Context)
 * @note  必须作为持久变量存在（不能是普通函数的局部栈变量）。
 * 推荐放在任务的私有数据结构体中。
 */
typedef struct
{
    uint16_t magic; /**< 初始化指纹，用于自动初始化识别 */
    uint32_t step;  /**< 当前执行的状态步进号或行号游标 */
    time_gt timer;  /**< 非阻塞延时的起点时间戳记录 */
} fsm_ctx_t;

/* ========================================================================== */
/* 核心控制宏 API                                                             */
/* ========================================================================== */

/**
 * @brief 软复位状态机
 * @param ctx 状态机上下文指针
 * @note  下一次调度时，状态机将从 case 0 重新开始执行
 */
#define fsm_reset(ctx)                                                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        (ctx)->magic = 0;                                                                                              \
    } while (0)

/**
 * @brief 开启状态机主干 (包含自动初始化机制)
 * @param ctx 状态机上下文指针
 */
#define _FSM_START(ctx)                                                                                                \
    time_gt _fsm_now = gmp_base_get_system_tick();                                                                     \
    if ((ctx)->magic != FSM_MAGIC_WORD)                                                                                \
    {                                                                                                                  \
        (ctx)->step = 0;                                                                                               \
        (ctx)->timer = 0;                                                                                              \
        (ctx)->magic = FSM_MAGIC_WORD;                                                                                 \
    }                                                                                                                  \
    switch ((ctx)->step)                                                                                               \
    {

/**
 * @brief 定义一个状态分支
 * @param n 分支编号 (必须是常量整数，且不可 >= FSM_LINE_OFFSET)
 */
#define _CASE_START(n) case (n):

/**
 * @brief 结束当前状态分支
 */
#define _CASE_END break;

/**
 * @brief 状态跳转 (非阻塞)
 * @param ctx 状态机上下文指针
 * @param n   目标状态编号
 * @note  调用后立即交出 CPU 控制权，下一次进入时直接执行目标状态
 */
#define fsm_goto(ctx, n)                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        (ctx)->step = (n);                                                                                             \
        return FSM_RET_YIELD;                                                                                          \
    } while (0)

/**
 * @brief 非阻塞延时
 * @param ctx 状态机上下文指针
 * @param ms  需要延时的毫秒数
 * @note  在延时期间，函数会不断返回 FSM_RET_YIELD
 */
#define fsm_wait(ctx, ms)                                                                                              \
    do                                                                                                                 \
    {                                                                                                                  \
        (ctx)->timer = _fsm_now;                                                                                       \
        (ctx)->step = (__LINE__ + FSM_LINE_OFFSET);                                                                    \
        return FSM_RET_YIELD;                                                                                          \
    case (__LINE__ + FSM_LINE_OFFSET):                                                                                 \
        if (gmp_base_time_sub(_fsm_now, (ctx)->timer) < (ms))                                                          \
        {                                                                                                              \
            return FSM_RET_YIELD;                                                                                      \
        }                                                                                                              \
    } while (0)

/* ========================================================================== */
/* 结束与异常捕获 API                                                         */
/* ========================================================================== */

/**
 * @brief 标准结束宏 (自带静默防跑飞保护)
 * @param ctx 状态机上下文指针
 * @note  如果状态跑飞，会自动复位上下文并返回 FSM_RET_ERROR
 */
#define _FSM_END(ctx)                                                                                                  \
    default:                                                                                                           \
        (ctx)->magic = 0;                                                                                              \
        (ctx)->step = 0;                                                                                               \
        return FSM_RET_ERROR;                                                                                          \
        }                                                                                                              \
        (ctx)->step = 0;                                                                                               \
        return FSM_RET_DONE;

/**
 * @brief 高级用法：自定义异常捕获起点
 * @param ctx 状态机上下文指针
 * @note  配合 _FSM_END_CATCH 使用，允许用户在状态跑飞时执行自定义急救代码
 */
#define _FSM_CATCH(ctx)                                                                                                \
    default:                                                                                                           \
        (ctx)->magic = 0;                                                                                              \
        (ctx)->step = 0;

/**
 * @brief 高级用法：自定义异常捕获终点
 */
#define _FSM_END_CATCH                                                                                                 \
    }                                                                                                                  \
    return FSM_RET_DONE;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_PM_DUFF_FSM_H_

/* example

// 1. 定义包含 FSM 上下文的业务结构体
typedef struct {
    fsm_ctx_t fsm;   // 状态机核心组件
    uint8_t count;   // 业务私有变量
} my_task_data_t;

// 2. 编写符合调度器接口的任务函数
gmp_task_status_t example_task(gmp_task_t* tsk)
{
    my_task_data_t* data = (my_task_data_t*)tsk->user_data;
    fsm_ctx_t* ctx = &data->fsm;
    
    _FSM_START(ctx)
    
    _CASE_START(0)
        gmp_base_print("Step 0: Initializing...\r\n");
        fsm_wait(ctx, 100);
        fsm_goto(ctx, 1);
    _CASE_END
    
    _CASE_START(1)
        data->count++;
        gmp_base_print("Step 1: Count = %d\r\n", data->count);
        fsm_wait(ctx, 500);
        fsm_goto(ctx, 2);
    _CASE_END
    
    _CASE_START(2)
        gmp_base_print("Step 2: Processing...\r\n");
        fsm_wait(ctx, 200);
        if (data->count >= 5) {
            data->count = 0; // 重置计数器
            fsm_goto(ctx, 0); 
        } else {
            fsm_goto(ctx, 1); 
        }
    _CASE_END
    
    // 我们在这里使用基础的 _FSM_END 即可，它会自动处理跑飞异常
    _FSM_END(ctx)
}

*/
