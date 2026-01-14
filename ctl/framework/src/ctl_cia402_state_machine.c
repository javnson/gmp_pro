/**
 * @file cia402_state_machine.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2026-01-14
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

#include <ctl/framework/cia402_state_machine.h>

/**
 * @brief 解析控制字 (0x6040) 并返回对应的命令枚举
 * 基于 CiA 402 State Transition 逻辑表
 * * @param control_word 16位控制字
 * @return cia402_cmd 解析出的命令
 */
cia402_cmd_t get_cia402_control_cmd(uint16_t control_word)
{
    // 1. Fault Reset (Bit 7)
    // 这是一个特殊的动作位，通常检测上升沿。
    // 但如果作为静态命令解析，只要置位即视为复位请求。
    // target state is CIA402_SM_FAULT_REACTIVE and then FAULT
    if ((control_word & 0x0080) != 0)
    {
        return CIA402_CMD_FAULT_RESET;
    }

    // 2. Disable Voltage (Bit 1 = 0)
    // 掩码: xxxx xxxx xxxx xx0x
    // 表格逻辑: 只要 Bit 1 为 0，就是 Disable Voltage (Transition 7,9,10,12)
    // 这是最高优先级的停机命令。
    // target state is Switch On Disabled
    if ((control_word & 0x0002) == 0)
    {
        return CIA402_CMD_DISABLE_VOLTAGE;
    }

    // 3. Quick Stop (Bit 2 = 0)
    // 掩码: xxxx xxxx xxxx x01x
    // 前提: Bit 1 必须为 1 (上面已经判断过了)
    // 表格逻辑: Bit 2 为 0，Bit 1 为 1 => Quick Stop (Transition 7,10,11)
    // target state is CIA402_SM_QUICK_STOP_ACTIVE and then Switch On Disabled
    if ((control_word & 0x0004) == 0)
    {
        return CIA402_CMD_QUICK_STOP;
    }

    // 4. Shutdown (Bit 0 = 0)
    // 掩码: xxxx xxxx xxxx x110
    // 前提: Bit 1=1, Bit 2=1
    // 表格逻辑: Bit 0 为 0 => Shutdown (Transition 2,6,8)
    // target state is Ready to Switch On
    if ((control_word & 0x0001) == 0)
    {
        return CIA402_CMD_SHUTDOWN;
    }

    // 5. Enable Operation (Bit 3 = 1)
    // 掩码: 0000 0000 0000 1111 (0xF) -> 值必须为 0xF (1111)
    if ((control_word & 0x000F) == 0x000F)
    {
        return CIA402_CMD_ENABLE_OPERATION;
    }

    // 6. Switch On (Bit 3 = 0)
    // 掩码: 0000 0000 0000 1111 (0xF) -> 值必须为 0x7 (0111)
    if ((control_word & 0x000F) == 0x0007)
    {
        return CIA402_CMD_SWITCHON;
    }

    // 7. Disable Operation (通常也是 0x7, 但在 Operation Enabled 状态下处理)
    // 如果都不匹配，返回 NULL 或者 KEEP
    return CIA402_CMD_NULL;
}

/**
 * @brief 根据 StatusWord 解析当前 CiA 402 状态
 * * @param status_word 16位的原始状态字 (0x6041)
 * @return cia402_state_t 对应的枚举状态
 */
cia402_state_t get_cia402_state(uint16_t status_word)
{
    // ---------------------------------------------------------
    // 掩码定义 (基于表格中的 x 位)
    // ---------------------------------------------------------

    // MASK_FULL: 关注 Bit 0, 1, 2, 3, 5, 6
    // 二进制: 0000 0000 0110 1111 -> 0x006F
    // 用于判断: Ready, Switched On, Op Enabled, Quick Stop
    const uint16_t MASK_FULL = 0x006F;

    // MASK_PARTIAL: 关注 Bit 0, 1, 2, 3, 6 (忽略 Bit 5)
    // 二进制: 0000 0000 0100 1111 -> 0x004F
    // 用于判断: Fault, Fault Reactive, Switch On Disabled, Not Ready
    const uint16_t MASK_PARTIAL = 0x004F;

    // ---------------------------------------------------------
    // 状态判定 (优先级顺序通常不影响结果，因为特征值是互斥的)
    // ---------------------------------------------------------

    // 1. (7) Fault Reactive: x0xx 1111 (Bit 6=0, Bit 5=x, Bits 0-3=1)
    if ((status_word & MASK_PARTIAL) == 0x000F)
    {
        return CIA402_SM_FAULT_REACTION;
    }

    // 2. (8) Fault: x0xx 1000 (Bit 6=0, Bit 5=x, Bit 3=1)
    if ((status_word & MASK_PARTIAL) == 0x0008)
    {
        return CIA402_SM_FAULT;
    }

    // 3. (5) Operation Enabled: x01x 0111 (Bit 6=0, Bit 5=1, Bit 3=0, Bit 2=1, Bit 1=1, Bit 0=1)
    if ((status_word & MASK_FULL) == 0x0027)
    {
        return CIA402_SM_OPERATION_ENABLED;
    }

    // 4. (4) Switched On: x01x 0011 (Bit 6=0, Bit 5=1, Bit 3=0, Bit 2=0, Bit 1=1, Bit 0=1)
    if ((status_word & MASK_FULL) == 0x0023)
    {
        return CIA402_SM_SWITCHED_ON;
    }

    // 5. (3) Ready to Switch On: x01x 0001 (Bit 6=0, Bit 5=1, Bit 3=0, Bit 2=0, Bit 1=0, Bit 0=1)
    if ((status_word & MASK_FULL) == 0x0021)
    {
        return CIA402_SM_READY_TO_SWITCH_ON;
    }

    // 6. (6) Quick Stop Active: x00x 0111 (Bit 6=0, Bit 5=0, Bit 3=0, Bit 2=1, Bit 1=1, Bit 0=1)
    // 注意: 这里 Bit 5 (Quick Stop) 为 0，表示正在急停中
    if ((status_word & MASK_FULL) == 0x0007)
    {
        return CIA402_SM_QUICK_STOP_ACTIVE;
    }

    // 7. (2) Switch On Disabled: x1xx 0000 (Bit 6=1, Bit 5=x, Bits 0-3=0)
    if ((status_word & MASK_PARTIAL) == 0x0040)
    {
        return CIA402_SM_SWITCH_ON_DISABLED;
    }

    // 8. (1) Not Ready to Switch On: x0xx 0000 (Bit 6=0, Bit 5=x, Bits 0-3=0)
    if ((status_word & MASK_PARTIAL) == 0x0000)
    {
        return CIA402_SM_NOT_READY_TO_SWITCH_ON;
    }

    // 如果都不匹配 (可能处于中间过渡态，通常归类为 Not Ready 或 Unknown)
    return CIA402_SM_UNKNOWN; // 或者 return CIA402_SM_NOT_READY_TO_SWITCH_ON;
}

// init cia402 state machine structure, state machine will switch to Not ready to switch on.
void init_cia402_state_machine(cia402_sm_t* sm)
{
    gmp_base_assert(sm);

    // init state
    sm->current_state = CIA402_SM_NOT_READY_TO_SWITCH_ON;
    sm->control_word.all = 0;
    sm->state_word.all = 0;

    sm->flag_fault_reset_request = 0;
    sm->flag_enable_control_word = 1; // 默认使能控制字
    sm->last_cb_result = CIA402_EC_KEEP;
    sm->last_fault_reset_bit = 0;

    // 初始化 Function Pointers 为 NULL (用户后续需要手动赋值)
    sm->switch_on_disabled = 0;
    sm->ready_to_switch_on = 0;
    sm->switched_on = 0;
    sm->operation_enabled = 0;
    sm->quick_stop_active = 0;
    sm->fault_reaction = 0;
    sm->fault = 0;

    // init default callback function
    sm->switch_on_disabled = default_cb_fn_switch_on_disabled;
    sm->ready_to_switch_on = default_cb_fn_ready_to_switch_on;
    sm->switched_on = default_cb_fn_switched_on;
    sm->operation_enabled = default_cb_fn_switched_on;
    sm->quick_stop_active = default_cb_fn_quick_stop_active;
    sm->fault_reaction = default_cb_fn_fault_reaction;
    sm->fault = default_cb_fn_fault;
}

void cia402_update_status_word(cia402_sm_t* sm)
{
    gmp_base_assert(sm);

    // 清除与状态相关的核心位: Bit 0,1,2,3,5,6
    // 保留 Bit 4 (Voltage), Bit 7 (Warning) 等，因为这些可能由外部逻辑设置
    // 这里为了简化，我们重写核心状态位

    cia402_state_word_t s = sm->state_word;

    // 默认值：QuickStop=1 (正常), Fault=0, SwitchOnDisabled=0
    s.all &= ~0x006F;                    // 清除 Bit 0,1,2,3,5,6
    s.all |= CIA402_STATEWORD_QUICKSTOP; // Bit 5 默认为 1 (Not Active)

    switch (sm->current_state)
    {
    case CIA402_SM_NOT_READY_TO_SWITCH_ON:
        // x0xx 0000
        // 通常初始化时 QuickStop 位也为 0
        s.bits.quick_stop = 0;
        break;

    case CIA402_SM_SWITCH_ON_DISABLED:
        // x1xx 0000
        s.bits.switch_on_disabled = 1;
        break;

    case CIA402_SM_READY_TO_SWITCH_ON:
        // x01x 0001
        s.bits.ready_to_switch_on = 1;
        break;

    case CIA402_SM_SWITCHED_ON:
        // x01x 0011
        s.bits.ready_to_switch_on = 1;
        s.bits.switched_on = 1;
        break;

    case CIA402_SM_OPERATION_ENABLED:
        // x01x 0111
        s.bits.ready_to_switch_on = 1;
        s.bits.switched_on = 1;
        s.bits.operation_enabled = 1;
        break;

    case CIA402_SM_QUICK_STOP_ACTIVE:
        // x00x 0111
        s.bits.quick_stop = 0;
        s.bits.ready_to_switch_on = 1;
        s.bits.switched_on = 1;
        s.bits.operation_enabled = 1;
        break;

    case CIA402_SM_FAULT_REACTION:
        // x0xx 1111 (Fault 但不改变当前系统状态)
        // Fault Reaction 期间，Bit 0-2 通常保持为 1 (看起来像 Op Enabled)，同时 Bit 3 (Fault) 置位
        s.bits.fault = 1;
        s.bits.ready_to_switch_on = 1;
        s.bits.switched_on = 1;
        s.bits.operation_enabled = 1;
        break;

    case CIA402_SM_FAULT:
        // x0xx 1000 (Fault only)
        s.bits.fault = 1;
        break;

    default:
        break;
    }

    // 更新回结构体
    sm->state_word.all = s.all;
}

// Fault condition cannot escape by this function
void cia402_transit(cia402_sm_t* sm, cia402_state_t next_state)
{
    if (sm->current_state != next_state && sm->current_state != CIA402_SM_FAULT &&
        sm->current_state != CIA402_SM_FAULT_REACTION)
    {
        // change to next state
        sm->current_state = next_state;

        // start a new delay stage
        sm->flag_delay_stage = 0;
        sm->entry_state_tick = gmp_base_get_system_tick();
        sm->current_state_counter = 0;

        // update state word here
        cia402_update_status_word(sm);
    }
}

static void _switch_on_disable_routine(cia402_sm_t* sm)
{
    gmp_base_assert(sm->switch_on_disabled);

    sm->last_cb_result = sm->switch_on_disabled(sm);

    // error condition
    if (sm->last_cb_result <= CIA402_EC_ERROR)
    {
        cia402_fault_request(sm);
        return;
    }

    // request next state
    if (sm->last_cb_result >= CIA402_EC_NEXT_STATE)
    {
#if defined CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH
        if (sm->current_cmd == CIA402_CMD_SHUTDOWN || sm->current_cmd == CIA402_CMD_SWITCHON ||
            sm->current_cmd == CIA402_CMD_ENABLE_OPERATION)
#else
        if (sm->current_cmd == CIA402_CMD_SHUTDOWN)
#endif // CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH
        {
            // The first time enter ready state, log current tick.
            if (sm->flag_delay_stage == 0)
            {
                sm->state_ready_tick = sm->current_tick;
                sm->flag_delay_stage = 1;
            }

            // judge if delay condition is meet.
            if (sm->current_tick - sm->state_ready_tick >= sm->minimum_transit_delay[1])
                cia402_transit(sm, CIA402_SM_READY_TO_SWITCH_ON);
        }
        return;
    }

    if (sm->last_cb_result == CIA402_EC_KEEP)
    {
        sm->flag_delay_stage = 0;
    }
}

static void _ready_to_switch_on_routine(cia402_sm_t* sm)
{
    gmp_base_assert(sm->ready_to_switch_on);

    sm->last_cb_result = sm->ready_to_switch_on(sm);

    // fault condition
    if (sm->last_cb_result <= CIA402_EC_ERROR)
    {
        sm->flag_delay_stage = 0;

        // switch to CIA402_SM_FAULT_REACTIVE
        cia402_fault_request(sm);

        return;
    }

    // request back state
    if (sm->current_cmd == CIA402_CMD_DISABLE_VOLTAGE)
    {
        cia402_transit(sm, CIA402_SM_SWITCH_ON_DISABLED);
        return;
    }

    // request next state
    if (sm->last_cb_result >= CIA402_EC_NEXT_STATE)
    {
#if defined CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH
        if (sm->current_cmd == CIA402_CMD_SWITCHON || sm->current_cmd == CIA402_CMD_ENABLE_OPERATION)
#else
        if (sm->current_cmd == CIA402_CMD_SWITCHON)
#endif // CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH
        {
            // The first time enter ready state, log current tick.
            if (sm->flag_delay_stage == 0)
            {
                sm->state_ready_tick = sm->current_tick;
                sm->flag_delay_stage = 1;
            }

            // judge if delay condition is meet.
            if (sm->current_tick - sm->state_ready_tick >= sm->minimum_transit_delay[2])
                cia402_transit(sm, CIA402_SM_SWITCHED_ON);
        }

        return;
    }

    if (sm->last_cb_result == CIA402_EC_KEEP)
    {
        sm->flag_delay_stage = 0;
    }
}

static void _switched_on_routine(cia402_sm_t* sm)
{
    gmp_base_assert(sm->switched_on);

    sm->last_cb_result = sm->switched_on(sm);

    // fault condition
    if (sm->last_cb_result <= CIA402_EC_ERROR)
    {
        cia402_fault_request(sm);
        return;
    }

    // request back state
    if (sm->current_cmd == CIA402_CMD_DISABLE_VOLTAGE)
    {
        cia402_transit(sm, CIA402_SM_SWITCH_ON_DISABLED);
        return;
    }

    if (sm->current_cmd == CIA402_CMD_SHUTDOWN)
    {
        cia402_transit(sm, CIA402_SM_READY_TO_SWITCH_ON);
        return;
    }

    // request next state
    if (sm->last_cb_result >= CIA402_EC_NEXT_STATE)
    {
        if (sm->current_cmd == CIA402_CMD_ENABLE_OPERATION)
        {
            // The first time enter ready state, log current tick.
            if (sm->flag_delay_stage == 0)
            {
                sm->state_ready_tick = sm->current_tick;
                sm->flag_delay_stage = 1;
            }

            // judge if delay condition is meet.
            if (sm->current_tick - sm->state_ready_tick >= sm->minimum_transit_delay[3])
                cia402_transit(sm, CIA402_SM_OPERATION_ENABLED);
        }

        return;
    }

    if (sm->last_cb_result == CIA402_EC_KEEP)
    {
        sm->flag_delay_stage = 0;
    }
}

static void _operation_enabled_routine(cia402_sm_t* sm)
{
    gmp_base_assert(sm->operation_enabled);

    sm->last_cb_result = sm->operation_enabled(sm);

    // fault condition
    if (sm->last_cb_result <= CIA402_EC_ERROR)
    {
        cia402_fault_request(sm);
        return;
    }

    // request back state
    if (sm->current_cmd == CIA402_CMD_DISABLE_VOLTAGE)
    {
        cia402_transit(sm, CIA402_SM_SWITCH_ON_DISABLED);
        return;
    }

    if (sm->current_cmd == CIA402_CMD_SHUTDOWN)
    {
        cia402_transit(sm, CIA402_SM_READY_TO_SWITCH_ON);
        return;
    }

    if (sm->current_cmd == CIA402_CMD_SWITCHON)
    {
        // 这里 Switch On 指令等同于 Disable Operation
        cia402_transit(sm, CIA402_SM_SWITCHED_ON);
    }

    // for now sm->last_cb_result >= CIA402_EC_NEXT_STATE or sm->last_cb_result == CIA402_EC_KEEP

    // request quick stop
    if (sm->current_cmd == CIA402_CMD_QUICK_STOP)
    {
        cia402_transit(sm, CIA402_SM_QUICK_STOP_ACTIVE);
    }
}

static void _quick_stop_active_routine(cia402_sm_t* sm)
{
    gmp_base_assert(sm->quick_stop_active);

    sm->last_cb_result = sm->quick_stop_active(sm);

    // fault condition
    if (sm->last_cb_result <= CIA402_EC_ERROR)
    {
        cia402_fault_request(sm);
        return;
    }

    // after complete quick stop routine
    if (sm->last_cb_result >= CIA402_EC_NEXT_STATE)
    {
        cia402_transit(sm, CIA402_SM_SWITCH_ON_DISABLED);
    }
}

static void _fault_reaction_routine(cia402_sm_t* sm)
{
    gmp_base_assert(sm->fault_reaction);

    sm->last_cb_result = sm->fault_reaction(sm);

    // after complete quick stop routine
    if (sm->last_cb_result >= CIA402_EC_NEXT_STATE)
    {
        cia402_transit(sm, CIA402_SM_FAULT);
    }
}

static void _fault_routine(cia402_sm_t* sm)
{
    gmp_base_assert(sm->fault);

    sm->last_cb_result = sm->fault(sm);

    // 必须且只能通过这一标志位请求复位，单稳态
    if (sm->flag_fault_reset_request)
    {
        sm->current_state = CIA402_SM_SWITCH_ON_DISABLED;

        // start a new delay stage
        sm->flag_delay_stage = 0;
        sm->entry_state_tick = gmp_base_get_system_tick();
        sm->current_state_counter = 0;

        // update state word here
        cia402_update_status_word(sm);

        sm->flag_fault_reset_request = 0;
    }
}

void cia402_fault_request(cia402_sm_t* sm)
{
    // 必须在正式进入错误状态之前清除之前设置的复位标志，防止系统错误恢复
    sm->flag_fault_reset_request = 0;

    // 标志进入新的一轮delay
    sm->flag_delay_stage = 0;

    // 切换
    sm->current_state = CIA402_SM_FAULT_REACTION;

    // 在这个函数中将会立即切换到CIA402_SM_FAULT_REACTION并立即执行一次fault_reaction函数。
    _fault_reaction_routine(sm);

    // update state word here
    cia402_update_status_word(sm);
}

// dispatch routine in mainloop
// This function would be called in mainloop
void dispatch_cia402_state_machine(cia402_sm_t* sm)
{
    gmp_base_assert(sm);

    sm->current_tick = gmp_base_get_system_tick();

    // snap shot of control word
    cia402_ctrl_word_t control_word = sm->control_word;

    // 1. get control command, get control word
    if (sm->flag_enable_control_word)
    {
        // get request state by control word
        sm->current_cmd = get_cia402_control_cmd(control_word.all);
    }

    // 2. judge if fault reset is request
    // 只有在falut状态下reset才是恢复到CIA402_SM_SWITCH_ON_DISABLED，其他的都通过request_state恢复。

    // Detection of Fault Reset Edge (0 -> 1)
    if (sm->flag_enable_control_word)
    {
        fast_gt current_reset_bit = control_word.bits.fault_reset;
        if (current_reset_bit && !sm->last_fault_reset_bit)
        {
            sm->flag_fault_reset_request = 1;
        }
        sm->last_fault_reset_bit = current_reset_bit;
    }

    //  3. if in other states Call CiA402 callback function
    switch (sm->current_state)
    {
    case CIA402_SM_NOT_READY_TO_SWITCH_ON:
        // transit to DISABLED, no callback function, all codes should implement in init stage.
        cia402_transit(sm, CIA402_SM_SWITCH_ON_DISABLED);
        break;

    case CIA402_SM_SWITCH_ON_DISABLED:
        _switch_on_disable_routine(sm);
        break;

    case CIA402_SM_READY_TO_SWITCH_ON:
        _ready_to_switch_on_routine(sm);
        break;

    case CIA402_SM_SWITCHED_ON:
        _switched_on_routine(sm);
        break;

    case CIA402_SM_OPERATION_ENABLED:
        _operation_enabled_routine(sm);
        break;

    case CIA402_SM_QUICK_STOP_ACTIVE:
        _quick_stop_active_routine(sm);
        break;

    case CIA402_SM_FAULT_REACTION:
        _fault_reaction_routine(sm);
        break;

    case CIA402_SM_FAULT:
        _fault_routine(sm);
        break;

        // unknown state
    default:
        cia402_fault_request(sm);
    }

    // 4. update status word
    cia402_update_status_word(sm);
}

// default callback function

//cia402_sm_error_code_t default_cb_fn_switch_on_disabled(cia402_sm_t* sm)
//{
//    (void)sm;
//
//    return CIA402_EC_NEXT_STATE;
//}
//
//cia402_sm_error_code_t default_cb_fn_ready_to_switch_on(cia402_sm_t* sm)
//{
//    (void)sm;
//
//    return CIA402_EC_NEXT_STATE;
//}
//
//cia402_sm_error_code_t default_cb_fn_switched_on(cia402_sm_t* sm)
//{
//    (void)sm;
//
//    return CIA402_EC_NEXT_STATE;
//}
//
//cia402_sm_error_code_t default_cb_fn_operation_enabled(cia402_sm_t* sm)
//{
//    (void)sm;
//
//    return CIA402_EC_KEEP;
//}
//
//cia402_sm_error_code_t default_cb_fn_quick_stop_active(cia402_sm_t* sm)
//{
//    (void)sm;
//
//    return CIA402_EC_NEXT_STATE;
//}
//
//cia402_sm_error_code_t default_cb_fn_fault_reaction(cia402_sm_t* sm)
//{
//    (void)sm;
//
//    return CIA402_EC_NEXT_STATE;
//}
//
//cia402_sm_error_code_t default_cb_fn_fault(cia402_sm_t* sm)
//{
//    (void)sm;
//
//    return CIA402_EC_KEEP;
//}

// 定义一些超时阈值 (单位 ms，根据实际情况调整)
#define TIMEOUT_PRECHARGE_MS 3000
#define TIMEOUT_ADC_CALIB_MS 1000
#define TIMEOUT_ALIGNMENT_MS 5000

// =========================================================================
// 1. Switch On Disabled (初始化与待机)
// =========================================================================
cia402_sm_error_code_t default_cb_fn_switch_on_disabled(cia402_sm_t* sm)
{
    // [Entry Action] 进入状态的第一拍执行
    if (sm->current_state_counter == 0)
    {
        // output_disable & power_off
        ctl_disable_pwm();
        cel_disable_main_contactor();
        ctl_disable_grid_relay();
        ctl_disable_precharge_relay();
        ctl_restore_brake(); // 抱闸
    }

    // [Do Action] ctl_if_adc_calibrate
    // 执行 ADC 校准 (非阻塞)
    if (ctl_exec_adc_calibration() == 0)
    {
        // 检查是否超时 (利用 entry_state_tick)
        if ((sm->current_tick - sm->entry_state_tick) > TIMEOUT_ADC_CALIB_MS)
        {
            return CIA402_EC_ERROR; // 校准超时，报错
        }
        return CIA402_EC_KEEP; // 等待校准完成
    }

    // 检查编码器/传感器状态
    if (ctl_check_encoder() == 0)
    {
        return CIA402_EC_KEEP; // 传感器未就绪
    }

    // 硬件准备就绪
    return CIA402_EC_NEXT_STATE;
}

// =========================================================================
// 2. Ready to Switch On (预充电与高压建立)
// =========================================================================
cia402_sm_error_code_t default_cb_fn_ready_to_switch_on(cia402_sm_t* sm)
{
    // [Entry Action] power_on (Start Precharge)
    if (sm->current_state_counter == 0)
    {
        // 确保主接触器断开
        cel_disable_main_contactor();
        // 闭合预充继电器
        ctl_enable_precharge_relay();

        // 重置内部阶段标志
        sm->flag_delay_stage = 0;
    }

    // [Do Action] 分阶段执行高压建立流程

    // Stage 0: 等待母线电压建立
    if (sm->flag_delay_stage == 0)
    {
        if (ctl_exec_dc_voltage_ready() == 1)
        {
            sm->flag_delay_stage = 1; // 电压OK，进入下一阶段
        }
        else
        {
            // 超时检测
            if ((sm->current_tick - sm->entry_state_tick) > TIMEOUT_PRECHARGE_MS)
            {
                return CIA402_EC_ERROR; // 预充超时
            }
            return CIA402_EC_KEEP;
        }
    }

    // Stage 1: 闭合主接触器 & 检查 PLL
    if (sm->flag_delay_stage == 1)
    {
        cel_enable_main_contactor();
        ctl_disable_precharge_relay(); // 切除预充

        // 如果是并网设备，检查 PLL
        if (ctl_check_pll_locked() == 1)
        {
            sm->flag_delay_stage = 2;
        }
        else
        {
            // 可以在这里加一个 PLL 锁定超时判断
            return CIA402_EC_KEEP;
        }
    }

    // output_disable (再次确认)
    ctl_disable_pwm();

    // 检查最小保持时间 (Minimum Dwell Time)
    // 注意：此处检查的是 minimum_transit_delay[2] (对应 ReadyToSwitchOn)
    if ((sm->current_tick - sm->state_ready_tick) < sm->minimum_transit_delay[2])
    {
        return CIA402_EC_KEEP;
    }

    return CIA402_EC_NEXT_STATE;
}

// =========================================================================
// 3. Switched On (物理连接与参数辨识)
// =========================================================================
cia402_sm_error_code_t default_cb_fn_switched_on(cia402_sm_t* sm)
{
    // [Entry Action] ctl_online_ready (Grid Relay or Alignment)
    if (sm->current_state_counter == 0)
    {
        // power_on: 保持高压在线
        cel_enable_main_contactor();

        // 闭合交流并网继电器
        ctl_enable_grid_relay();
    }

    // [Do Action] 转子定位 / 对齐
    // 注意：output_disable 在此阶段通常意味着 PWM 不发波，或者仅发定位直流向量
    // 如果 ctl_exec_rotor_alignment 内部会操作 PWM，则这里不需要显式 disable

    if (ctl_exec_rotor_alignment() == 0)
    {
        if ((sm->current_tick - sm->entry_state_tick) > TIMEOUT_ALIGNMENT_MS)
        {
            return CIA402_EC_ERROR;
        }
        return CIA402_EC_KEEP;
    }

    // 检查最小保持时间
    if ((sm->current_tick - sm->state_ready_tick) < sm->minimum_transit_delay[3])
    {
        return CIA402_EC_KEEP;
    }

    return CIA402_EC_NEXT_STATE;
}

// =========================================================================
// 4. Operation Enabled (闭环运行)
// =========================================================================
cia402_sm_error_code_t default_cb_fn_operation_enabled(cia402_sm_t* sm)
{
    // [Entry Action] output_enable
    if (sm->current_state_counter == 0)
    {
        ctl_enable_pwm();    // 开启 PWM
        ctl_release_brake(); // 松开抱闸
    }

    // [Do Action] 实时安规检查
    if (ctl_check_compliance() == 0)
    {
        // 运行中发生安规故障（电网掉电、过流等）
        return CIA402_EC_ERROR;
    }

    // 正常运行，保持状态
    return CIA402_EC_KEEP;
}

// =========================================================================
// 5. Quick Stop Active (快速停机)
// =========================================================================
cia402_sm_error_code_t default_cb_fn_quick_stop_active(cia402_sm_t* sm)
{
    // [Entry Action] power_off sequence start
    if (sm->current_state_counter == 0)
    {
        // output_disable: 立即封波或开始减速
        // 这里默认实现为立即封波
        ctl_disable_pwm();
        ctl_restore_brake();
    }

    // [Do Action] 等待停机完成 (如果是有减速过程的)
    // 这里简化为立即完成

    // power_off: 停机完成后，逻辑上已经 Power Off 了

    return CIA402_EC_NEXT_STATE; // 跳转到 Switch On Disabled
}

// =========================================================================
// 6. Fault Reaction (故障反应)
// =========================================================================
cia402_sm_error_code_t default_cb_fn_fault_reaction(cia402_sm_t* sm)
{
    // [Entry Action] 安全第一
    if (sm->current_state_counter == 0)
    {
        // output_disable
        ctl_disable_pwm();
        ctl_restore_brake();

        // power_off: 切断外部连接
        ctl_disable_grid_relay();
    }

    // [Do Action] 可以在这里记录故障日志，或者等待电流衰减

    return CIA402_EC_NEXT_STATE; // 跳转到 Fault
}

// =========================================================================
// 7. Fault (故障停机态)
// =========================================================================
cia402_sm_error_code_t default_cb_fn_fault(cia402_sm_t* sm)
{
    // [Entry Action] 冗余安全切断
    if (sm->current_state_counter == 0)
    {
        ctl_disable_pwm();
        cel_disable_main_contactor(); // 彻底断高压
        ctl_disable_grid_relay();
        ctl_disable_precharge_relay();
    }

    // nothing happened. 等待 Reset 信号
    return CIA402_EC_KEEP;
}

