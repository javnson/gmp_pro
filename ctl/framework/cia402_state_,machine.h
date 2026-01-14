
#ifndef _FILE_CIA402_STATE_MACHINE_H_
#define _FILE_CIA402_STATE_MACHINE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// 在CiA 402 标准，从 Switch On Disabled 必须收到 Shutdown (0x06) 才能进入 Ready to Switch On 。
// 直接发 0x0F 或 0x07 通常是被忽略的。
// 这个宏允许连续切换状态，直接进入0x0F状态，方便调试。
#define CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH

//////////////////////////////////////////////////////////////////////////
// Control word definition
// Control Word 6040h: "State Transition Commands" Bits
//

// 1: 允许切换到 Switched On 状态
#define CIA402_CONTROLWORD_SWITCHON (0x0001)

// 1: 允许直流母线/高压存在 (硬件使能)
#define CIA402_CONTROLWORD_ENABLE_VOLTAGE (0x0002)

// 0: 触发急停 (注意是低电平有效)；1: 正常工作
#define CIA402_CONTROLWORD_QUICKSTOP (0x0004)

// 1: 允许发波/运行；0: 封波/禁止运行
#define CIA402_CONTROLWORD_ENABLE_OPERATION (0x0008)

// 0 $\to$ 1 (上升沿): 复位故障
#define CIA402_CONTROLWORD_FAULT_RESET (0x0080)

// 1: 暂停运行 (但不退出 Operation Enabled)
#define CIA402_CONTROLWORD_HALT (0x0100)

typedef union tag_cia402_ctrl_word {
    uint16_t all;

    struct
    {
        // --- 字节 0 (低8位) ---
        uint16_t switch_on : 1;        // Bit 0: 开启 (Switch On)
        uint16_t enable_voltage : 1;   // Bit 1: 允许电压 (Enable Voltage)
        uint16_t quick_stop : 1;       // Bit 2: 快速停机 (Quick Stop) - 注意: 1表示正常，0表示急停
        uint16_t enable_operation : 1; // Bit 3: 允许运行 (Enable Operation)

        uint16_t oms_4 : 1; // Bit 4: 模式相关 (Operation mode specific)
        uint16_t oms_5 : 1; // Bit 5: 模式相关
        uint16_t oms_6 : 1; // Bit 6: 模式相关

        uint16_t fault_reset : 1; // Bit 7: 故障复位 (Fault Reset) - 上升沿有效

        // --- 字节 1 (高8位) ---
        uint16_t halt : 1;         // Bit 8: 暂停 (Halt)
        uint16_t oms_9 : 1;        // Bit 9: 模式相关 (Operation mode specific)
        uint16_t reserved : 1;     // Bit 10: 保留
        uint16_t manufacturer : 5; // Bits 11-15: 厂商自定义 (Manufacturer specific)
    } bits;
} cia402_ctrl_word_t;

typedef enum tag_cia402_cmd
{
    CIA402_CMD_NULL = 0,
    CIA402_CMD_DISABLE_VOLTAGE = 1,
    CIA402_CMD_SHUTDOWN,
    CIA402_CMD_SWITCHON,
    CIA402_CMD_ENABLE_OPERATION,
    CIA402_CMD_QUICK_STOP,
    CIA402_CMD_FAULT_RESET
} cia402_cmd_t;

#define CIA402_CMD_DISABLE_OPERATION CIA402_CMD_SWITCHON

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

//////////////////////////////////////////////////////////////////////////
// Status Word definition

// 1: 系统已预充完成，准备好合闸
#define CIA402_STATEWORD_READY_TO_SWITCH_ON (0x0001)

// 1: 强电回路已接通 (Relay Closed)
#define CIA402_STATEWORD_SWITCHED_ON (0x0002)

// 1: PWM 正在发波，系统正在运行
#define CIA402_STATEWORD_OPERATION_ENABLED (0x0004)

// 1: 发生故障
#define CIA402_STATEWORD_FAULT (0x0008)

// 1: 直流母线电压正常
#define CIA402_STATEWORD_VOLTAGE_ENABLED (0x0010)

// 0: 正在急停中；1: 正常 (注意逻辑反向)
#define CIA402_STATEWORD_QUICKSTOP (0x0020)

// 1: 系统处于禁止合闸状态 (通常是刚上电或故障复位后)
#define CIA402_STATEWORD_SWICH_ON_DISABLED (0X0040)

// 1: 允许远程控制
#define CIA402_STATEWORD_REMOTE (0x0200)

// 1: 目标值已达到 (速度/位置/电流稳定)
#define CIA402_STATEWORD_TARGET_REACHED (0x0400)

typedef union tag_cia402_state_word {
    uint16_t all;

    struct
    {
        // --- 低 8 位 (Byte 0) ---

        // Bit 0: 准备好合闸 (Ready to Switch On)
        // 1 = 系统已完成预充，无故障，等待 Switch On 命令
        uint16_t ready_to_switch_on : 1;

        // Bit 1: 已合闸 (Switched On)
        // 1 = 强电电路已接通 (继电器闭合)
        uint16_t switched_on : 1;

        // Bit 2: 运行允许 (Operation Enabled)
        // 1 = PWM 已输出，闭环控制正在运行
        uint16_t operation_enabled : 1;

        // Bit 3: 故障 (Fault)
        // 1 = 发生故障
        uint16_t fault : 1;

        // Bit 4: 电压允许 (Voltage Enabled)
        // 1 = 直流母线电压/主电源已施加
        uint16_t voltage_enabled : 1;

        // Bit 5: 快速停机 (Quick Stop)
        // 注意逻辑反向:
        // 1 = 正常 (Drive is NOT performing quick stop)
        // 0 = 正在急停 (Drive is reacting to a Quick Stop request)
        uint16_t quick_stop : 1;

        // Bit 6: 合闸禁止 (Switch On Disabled)
        // 1 = 系统处于初始化完成或故障复位后的待机状态，禁止直接合闸
        uint16_t switch_on_disabled : 1;

        // Bit 7: 警告 (Warning)
        // 1 = 有警告参数超限，但不需要停机
        uint16_t warning : 1;

        // --- 高 8 位 (Byte 1) ---

        // Bit 8: 厂商自定义 (Manufacturer specific)
        uint16_t manufacturer_8 : 1;

        // Bit 9: 远程控制 (Remote)
        // 1 = 控制权在 CANopen/EtherCAT 总线 (响应 0x6040)
        // 0 = 本地控制 (忽略总线控制字)
        uint16_t remote : 1;

        // Bit 10: 目标到达 (Target Reached)
        // 1 = 轴已停止或到达目标位置/速度
        // 在 Homing 模式下表示回零完成
        uint16_t target_reached : 1;

        // Bit 11: 内部限制有效 (Internal limit active)
        // 1 = 内部电流、速度或位置软限位已被触发
        uint16_t internal_limit_active : 1;

        // Bits 12-13: 模式相关 (Operation mode specific)
        // 例如在 CSP 模式下通常为 0，在 Homing 模式下指示状态
        uint16_t oms_12 : 1;
        uint16_t oms_13 : 1;

        // Bit 14: 厂商自定义 (这里您定义为回零完成标志)
        // 1 = Home has completed
        uint16_t mfg_home_completed : 1;

        // Bit 15: 厂商自定义
        uint16_t manufacturer_15 : 1;

    } bits;
} cia402_state_word_t;

// Not ready to switch on: Reset - self-test/initialization
// Switch on Disabled: Successfully initialization - Activate Communication
//

typedef enum tag_cia402_state
{
    // @brief Driver “HV” Power Disabled - if there is a provision to switch drive power.
    // Processor power on, drive initialization in progress, BRAKE on in this state, if present.
    // Reset - self-test/initialization
    CIA402_SM_NOT_READY_TO_SWITCH_ON = 0,

    // Driver “HV” Power Disabled - if there is a provision to switch drive power.
    // Processor power on, Initialization complete, drive parameters set up, drive disabled.
    // Successfully initialization - Activate Communication
    CIA402_SM_SWITCH_ON_DISABLED,

    // Ready to have High Voltage power applied.
    // Shutdown command has received.
    CIA402_SM_READY_TO_SWITCH_ON,

    // High Voltage enabled to driver, power amp ready, drive function disabled
    // Switch on command has received.
    CIA402_SM_SWITCHED_ON,

    // Power enabled to driver, no faults have been detected, drive function is enabled and there is power to motor. The servo is active.
    // Enable operation command has received.
    CIA402_SM_OPERATION_ENABLED,

    // Power enabled to driver, Quick stop function is being executed, drive function is enabled, and power is applied to motor.
    // Quick Stop command has received, execute Quick Stop function or transit to Switch On Disabled.
    CIA402_SM_QUICK_STOP_ACTIVE,

    // A fault has occurred in drive, quick stop being executed. Drive enabled and power are applied to drive while reacting.
    CIA402_SM_FAULT_REACTION,

    // A fault has occurred, high voltage MAY be switched off, and drive function is disabled.
    CIA402_SM_FAULT,

    CIA402_SM_UNKNOWN = 0xFF
} cia402_state_t;

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

struct _tag_cia402_state_machine;

typedef enum cia402_sm_error_code
{
    // Not ready
    CIA402_EC_KEEP = 0,

    // This state is ready if user request, it may change to next state
    // Hardware is ready, next is
    CIA402_EC_NEXT_STATE = 1,

    // fatal error happened, must skip to fault.
    CIA402_EC_ERROR = -1
} cia402_sm_error_code_t;

typedef cia402_sm_error_code_t (*cia402_cb_fn_t)(_tag_cia402_state_machine* sm);

typedef struct _tag_cia402_state_machine
{
    // WR Control Word 6040h
    cia402_ctrl_word_t control_word;

    // RO Status Word 6041h
    cia402_state_word_t state_word;

    // RO current state
    cia402_state_t current_state;

    // RO current command
    cia402_cmd_t current_cmd;

    // RO request target status
    cia402_state_t request_state;

    //
    // function handle
    //

    // when current state is switched on, the callback function would be called.
    cia402_cb_fn_t switch_on_disabled;

    cia402_cb_fn_t ready_to_switch_on;

    cia402_cb_fn_t switched_on;

    cia402_cb_fn_t operation_enabled;

    cia402_cb_fn_t quick_stop_active;

    cia402_cb_fn_t fault_reaction;

    cia402_cb_fn_t fault;

    //
    // Configuration
    //

    cia402_sm_error_code_t last_cb_result;

    // this flag would be cleared after reset process
    fast_gt flag_fault_reset_request;

    // if control word is enable
    fast_gt flag_enable_control_word;

    // entry delay stage
    fast_gt flag_delay_stage;

    // 分别对应前4个正常状态切换的最小延迟，用于保证接触器正确接触、母线电压稳定等
    // 当切换条件满足时需要最少达到下面的延时要求才可以切换到下一个状态
    // [0] CIA402_SM_NOT_READY_TO_SWITCH_ON 状态至少要保持的时间
    // [1] CIA402_STATEWORD_SWITCHED_ON 状态至少要保持的时间
    // [2] CIA402_SM_READY_TO_SWITCH_ON 状态至少要保持的时间
    // [3] CIA402_SM_SWITCHED_ON 状态至少要保持的时间
    // 其他状态不存在至少保持时间，在满足切换条件时马上切换
    time_gt minimum_transit_delay[4];

    // 当前状态建立后的时间
    time_gt state_ready_tick;

    time_gt current_tick;

    // 用于辅助判断是否收到错误复位的上升边沿
    fast_gt last_fault_reset_bit;

} cia402_sm_t;

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
}

static void cia402_update_status_word(cia402_sm_t* sm)
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

void cia402_fault_request(cia402_sm_t* sm)
{
    // 必须在正式进入错误状态之前清除之前设置的复位标志，防止系统错误恢复
    sm->flag_fault_reset_request = 0;

    // 标志进入新的一轮delay
    sm->flag_delay_stage = 0;

    // 切换
    sm->current_state = CIA402_SM_FAULT_REACTION;

    // 在这个函数中将会立即切换到CIA402_SM_FAULT_REACTION并立即执行一次fault_reaction函数。
    _fault_reaction_routine();

    // update state word here
    //...
}

// Fault condition cannot release by this function
void cia402_transit(cia402_sm_t* sm, cia402_state_t next_state)
{
    if (sm->current_state != next_state && sm->current_state != CIA402_SM_FAULT &&
        sm->current_state != CIA402_SM_FAULT_REACTION)
    {
        // change to next state
        sm->current_state = next_state;

        // start a new delay stage
        sm->flag_delay_stage = 0;
        //sm->state_entry_tick = gmp_base_get_system_tick();

        // update state word here
        cia402_update_status_word(sm);
    }
}

static void _switch_on_disable_routine(cia402_state_t* sm)
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
            if (sm->current_tick - sm->state_ready_tick >= sm->minimum_transit_delay[0])
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
    if (sm->current_cmd == CIA402_CMD_DISABLE_VOLTAGE || sm->current_cmd == CIA402_CMD_ENABLE_OPERATION)
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
            if (sm->current_tick - sm->state_ready_tick >= sm->minimum_transit_delay[0])
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
            if (sm->current_tick - sm->state_ready_tick >= sm->minimum_transit_delay[0])
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
        sm->flag_fault_reset_request = 0;
    }
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

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CIA402_STATE_MACHINE_H_
