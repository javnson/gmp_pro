
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

// 在初始化过程中暂时禁用控制字解析，可以直接施加命令
#define CIA402_CONFIG_DISABLE_CONTROL_WORD_DEFAULT

// 从Not ready to switch on -> ready to switch on的最小延时
// 这一个设计已经被禁用，不会真的有延时发生
#ifndef CIA402_CONFIG_MIN_DELAY_READY
#define CIA402_CONFIG_MIN_DELAY_READY (100)
#endif // CIA402_CONFIG_MIN_DELAY_READY

// 从Switched on disabled -> ready to switch on 的最小延时
#ifndef CIA402_CONFIG_MIN_DELAY_SHUTDOWN
#define CIA402_CONFIG_MIN_DELAY_SHUTDOWN (100)
#endif // CIA402_CONFIG_MIN_DELAY_SHUTDOWN

// 从Ready to switch on -> Switch on 的最小延时
#ifndef CIA402_CONFIG_MIN_DELAY_SWITCHON
#define CIA402_CONFIG_MIN_DELAY_SWITCHON (100)
#endif // CIA402_CONFIG_MIN_DELAY_SHUTDOWN

// 从Switch on -> Operation Enable的最小延时
#ifndef CIA402_CONFIG_MIN_DELAY_OPERATION_EN
#define CIA402_CONFIG_MIN_DELAY_OPERATION_EN (100)
#endif // CIA402_CONFIG_MIN_DELAY_OPERATION_EN

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
cia402_cmd_t get_cia402_control_cmd(uint16_t control_word);

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
cia402_state_t get_cia402_state(uint16_t status_word);

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

typedef cia402_sm_error_code_t (*cia402_cb_fn_t)(struct _tag_cia402_state_machine* sm);

typedef struct _tag_cia402_state_machine
{
    // WR Control Word 6040h
    cia402_ctrl_word_t control_word;

    // RO Status Word 6041h
    cia402_state_word_t state_word;

    // RO current state
    cia402_state_t current_state;

    // RO current command, WR if control word is disabled.
    cia402_cmd_t current_cmd;

    // RO request target status
    //cia402_state_t request_state;

    //
    // function handle
    //

    // when current state is switched on, the callback function would be called.
    cia402_cb_fn_t switch_on_disabled;

    // ready to have high voltage power applied.
    // DC power is enable
    //
    cia402_cb_fn_t ready_to_switch_on;

    // waiting for meeting enable condition
    // clear controller
    // PWM output is disabled
    cia402_cb_fn_t switched_on;

    // PWM output is enable
    cia402_cb_fn_t operation_enabled;

    // close PWM output
    cia402_cb_fn_t quick_stop_active;

    // just stop here
    cia402_cb_fn_t fault_reaction;

    cia402_cb_fn_t fault;

    //
    // Configuration
    //

    cia402_sm_error_code_t last_cb_result;

    // this flag would be cleared after reset process
    // user may set this member to 1, in order reset from fault, if control word is disabled.
    fast_gt flag_fault_reset_request;

    // if control word is enable
    fast_gt flag_enable_control_word;

    // entry delay stage
    fast_gt flag_delay_stage;

    // tick when enter current state
    time_gt entry_state_tick;

    // 表示当前状态已经运行了多少个周期
    uint32_t current_state_counter;

    // 分别对应前4个正常状态切换的最小延迟，用于保证接触器正确接触、母线电压稳定等
    // 当切换条件满足时需要最少达到下面的延时要求才可以切换到下一个状态
    // [0] CIA402_SM_NOT_READY_TO_SWITCH_ON 状态至少要保持的时间(disabled)
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
void init_cia402_state_machine(cia402_sm_t* sm);

// update status word by state machine
void cia402_update_status_word(cia402_sm_t* sm);

// Enter fault state
void cia402_fault_request(cia402_sm_t* sm);

// Fault condition cannot escape by this function
void cia402_transit(cia402_sm_t* sm, cia402_state_t next_state);

// dispatch routine in mainloop
// This function would be called in mainloop
void dispatch_cia402_state_machine(cia402_sm_t* sm);

//////////////////////////////////////////////////////////////////////////
// default callback function
//

// ctl_if_adc_calibrate function would be called in this function
// power_off function would be called in this function
// output_disable function would be called in this function
cia402_sm_error_code_t default_cb_fn_switch_on_disabled(cia402_sm_t* sm);

// power_on function would be called in this function
// output_disable function would be called in this function
cia402_sm_error_code_t default_cb_fn_ready_to_switch_on(cia402_sm_t* sm);

// power_on function would be called in this function
// output_disable function would be called in this function
// ctl_online_ready function would be called
cia402_sm_error_code_t default_cb_fn_switched_on(cia402_sm_t* sm);

// power_on function would be called in this function
// output_enable function would be called in this function
cia402_sm_error_code_t default_cb_fn_operation_enabled(cia402_sm_t* sm);

// power_off function would be called in this function
// output_disable function would be called in this function
cia402_sm_error_code_t default_cb_fn_quick_stop_active(cia402_sm_t* sm);

// power_off function would be called in this function
// output_disable function would be called in this function
cia402_sm_error_code_t default_cb_fn_fault_reaction(cia402_sm_t* sm);

// nothing happened.
cia402_sm_error_code_t default_cb_fn_fault(cia402_sm_t* sm);

//////////////////////////////////////////////////////////////////////////
// default functional prototype
// user should implement the following function in your own project
//

// =============================================================
// 1. 功率级执行 (Power Stage & Actuators)
// =============================================================

/**
 * @brief 硬件PWM输出使能/禁止
 * @note 在 Operation Enabled 状态下为 true，其他状态为 false
 * @param enable true: 开启PWM驱动; false: 封锁PWM（高阻态或特定电平）
 */

void ctl_enable_pwm();

void ctl_disable_pwm();
/**
 * @brief 主接触器/直流继电器控制
 * @note 通常在 Ready to Switch On 阶段闭合
 * @param close true: 吸合; false: 断开
 */

void ctl_enable_main_contactor();

void ctl_disable_main_contactor();

/**
 * @brief 预充电继电器控制
 * @note 在 Switch On Disabled -> Ready to Switch On 过渡期间使用
 */

void ctl_enable_precharge_relay();

void ctl_disable_precharge_relay();

// =============================================================
// 2. 采样与校准 (Sensing & Calibration)
// =============================================================

/**
 * @brief 执行ADC偏置校准
 * @note 通常在 Not Ready 或 Switch On Disabled 状态下调用
 * @return true: 校准完成且成功; false: 失败或正在进行中
 */
fast_gt ctl_exec_adc_calibration(void);

/**
 * @brief 检查直流母线电压是否在允许范围内
 * @note 用于 Ready to Switch On 的准入条件
 */
fast_gt ctl_exec_dc_voltage_ready(void);

// =============================================================
// 3. 机械制动 (Mechanical Brake)
// =============================================================

/**
 * @brief 机械抱闸控制 (Holding Brake)
 * @note 电机场景特有。通常 Operation Enabled 时 release (true)，否则 engage (false)
 * @param release true: 松开抱闸(允许转动); false: 抱死(制动)
 */
void ctl_release_brake();

void ctl_restore_brake();

// =============================================================
// 4. 编码器与对齐 (Position & Alignment)
// =============================================================

/**
 * @brief 检查编码器/位置传感器状态
 * @note 在 Ready to Switch On 之前必须通过
 */
fast_gt ctl_check_encoder(void);

/**
 * @brief 启动转子初始位置检测 (IPD / Alignment)
 * @note 针对同步电机。通常在 Switched On -> Operation Enabled 瞬间触发
 */
fast_gt ctl_exec_rotor_alignment(void);

// =============================================================
// 5. 电网同步 (Grid Synchronization)
// =============================================================

/**
 * @brief 启动/复位锁相环 (PLL)
 * @note 在 Ready to Switch On 状态下必须启动 PLL
 */
fast_gt ctl_check_pll_locked(void);

/**
 * @brief 检查电网电压/频率是否符合安规 (Grid Code)
 * @note 整个运行周期都需要检查
 */
fast_gt ctl_check_compliance(void);

// =============================================================
// 6. 交流侧操作 (AC Side Operations)
// =============================================================

/**
 * @brief 交流并网继电器/断路器控制
 * @note 在 Ready -> Switched On 跳转时闭合
 * @param close true: 并网; false: 解列
 */
void ctl_enable_grid_relay();

void ctl_disable_grid_relay();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CIA402_STATE_MACHINE_H_
