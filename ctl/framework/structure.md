#### 1. CiA 402 (CANopen Drives Profile) - *最推荐*

虽然它是为电机驱动设计的，但绝大多数现代伺服和变流器都沿用了这套逻辑。

- **特点**：定义了非常严谨的状态流转：
  - `Switch On Disabled` (初始化完成，禁止合闸)
  - `Ready To Switch On` (无故障，等待高压)
  - `Switched On` (高压已上，PWM未开)
  - `Operation Enabled` (PWM开启，闭环运行)
- **控制字 (ControlWord)**：0x6040，通过位操作控制状态跳转（如 Bit 0: Switch On, Bit 1: Enable Voltage, Bit 3: Enable Operation）。
- **状态字 (StatusWord)**：0x6041，反馈当前状态。
- **适用性**：非常适合做标准化的 Modbus/CAN/EtherCAT 通讯接口。



### 5. ControlWord 和 StatusWord 具体定义



这是 CiA 402 的灵魂。请务必严格遵守以下位定义，这样标准的 PLC 才能控制您的设备。

#### ControlWord (0x6040) - 上位机发给您的指令

| **Bit** | **名称**             | **作用**                                        |
| ------- | -------------------- | ----------------------------------------------- |
| **0**   | **Switch On**        | 1: 允许切换到 `Switched On` 状态                |
| **1**   | **Enable Voltage**   | 1: 允许直流母线/高压存在 (硬件使能)             |
| **2**   | **Quick Stop**       | **0**: 触发急停 (注意是低电平有效)；1: 正常工作 |
| **3**   | **Enable Operation** | 1: 允许发波/运行；0: 封波/禁止运行              |
| **7**   | **Fault Reset**      | 0 $\to$ 1 (上升沿): 复位故障                    |
| 4-6     | Mode Specific        | 根据运行模式不同而不同 (如原点回归、位置模式等) |
| 8       | Halt                 | 1: 暂停运行 (但不退出 Operation Enabled)        |

**典型命令序列 (十六进制):**

1. **0x0006 (Shutdown)**: 跳转到 `Ready to Switch On` (预充完成)。
2. **0x0007 (Switch On)**: 跳转到 `Switched On` (并网继电器闭合)。
3. **0x000F (Enable Op)**: 跳转到 `Operation Enabled` (PWM 开启)。

#### StatusWord (0x6041) - 您反馈给上位机的状态

| **Bit** | **名称**               | **作用**                                           |
| ------- | ---------------------- | -------------------------------------------------- |
| **0**   | **Ready to Switch On** | 1: 系统已预充完成，准备好合闸                      |
| **1**   | **Switched On**        | 1: 强电回路已接通 (Relay Closed)                   |
| **2**   | **Operation Enabled**  | 1: PWM 正在发波，系统正在运行                      |
| **3**   | **Fault**              | 1: 发生故障                                        |
| **4**   | **Voltage Enabled**    | 1: 直流母线电压正常                                |
| **5**   | **Quick Stop**         | 0: 正在急停中；1: 正常 (注意逻辑反向)              |
| **6**   | **Switch On Disabled** | 1: 系统处于禁止合闸状态 (通常是刚上电或故障复位后) |
| 9       | Remote                 | 1: 允许远程控制                                    |
| 10      | Target Reached         | 1: 目标值已达到 (速度/位置/电流稳定)               |

**典型状态反馈:**

- **0xxxx0**: Not Ready to Switch On
- **0x0250**: Switch On Disabled (Bit 6, 4 亮) -> 待机，有低压，无高压。
- **0x0231**: Ready to Switch On (Bit 5, 4, 0 亮) -> 预充好了。
- **0x0233**: Switched On (Bit 5, 4, 1, 0 亮) -> 继电器吸合了。
- **0x0237**: Operation Enabled (Bit 5, 4, 2, 1, 0 亮) -> 正在运行。



定义一个虚基类，其中包含如下几个虚函数，这些虚函数都是非阻塞（tick/polling）的，并且将会在主循环中被调用，定义如下:



(0) Not ready to switch on，这个过程在系统的init函数中执行，并不定义一个独立的虚函数。完成初始化后自动进入下一状态(1)。

(1) switch on disable，返回1进入ready to switch on，返回0保持此状态，返回-1进入错误状态；(在此过程中系统不断clear，控制系统不运行，调制模块被禁用只能输出0，pwm使能引脚处于不使能，接触器都不吸合，各故障复位)，进入下一步骤的条件，用户请求。

(2) ready to switch on，返回1进入switched on，返回0保持，返回-1进入错误状态；(控制系统不运行，安全管理模块正常运行，主接触器吸合，负载接触器不吸合，pwm输出不允许使能，调制模块禁用，始终输出0，故障信号不再复位，adc偏置校准)，锁相环必须在这一阶段使能，切换到下一状态的最小条件，母线电压正确建立。

(3) switch on，返回1进入operation enabled，返回0保持，返回-1进入错误状态；(控制系统不运行，观测器开始运行(比如电网的锁相环)，安全管理模块正常允许，主接触器吸合，pwm输出使能，调制模块使能，但是处于0位(对于电机控制场景为50%占空比输出)，负载接触器吸合)，切换到下一步骤的最小条件，锁相环收敛，adc校准完成。

(4) operation enabled，返回0保持，返回-1进入错误状态。(控制系统运行，观测器运行，安全模块运行，主接触器吸合，负载接触器吸合，pwm使能，pwm复位指令禁用(硬件保护不会被自动复位)，调制模块使能，根据当前的运行mode正常运行)

(5) fault / quick stop，返回0保持，返回-1保持，返回1退会switch on disable。



返回值：小于0全部为错误状态，等于0为保持状态，大于1表示允许切换到下一个状态。返回值使用枚举给出定义。



设计一个函数是sm_dispatch，保存成员control word和state word，这两个成员由sm_dispatch函数维护。当用户请求某一个状态时由sm_dispatch决定是否/如何切换。当请求状态向前时，必须一级级推进，当请求状态后退时可以直接退回。



另外，状态机保存成员operation mode，这个成员根据场景不同定义不同的工作状态，比如速度环，位置环，识别等。



还需要提供一组工具函数，包括请求某一个状态，理论上用户仅应当使用此函数请求状态机到某个状态。

对于一个特定的应用场景，应当派生此类，并实现以上这些虚函数。



``` C++
// 状态机动作反馈
enum class SmRet {
    KEEP = 0,
    NEXT_STATE = 1,
    ERROR = -1
};

// 状态枚举 (对应 CiA 402)
enum class Cia402State {
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    FAULT
};

class InverterStateMachine {
public:
    // --- 外部调用接口 (Dispatcher) ---
    // 在主循环中高频调用 (例如 10kHz 或 1kHz)
    void tick() {
        // 1. 处理控制字请求 (向前一步一步走，向后可以直接跳)
        handle_control_word_request();
        
        // 2. 执行当前状态的虚函数
        SmRet ret = SmRet::KEEP;
        switch (current_state_) {
            case Cia402State::SWITCH_ON_DISABLED: ret = on_switch_on_disabled(); break;
            case Cia402State::READY_TO_SWITCH_ON: ret = on_ready_to_switch_on(); break;
            case Cia402State::SWITCHED_ON:        ret = on_switched_on(); break;
            case Cia402State::OPERATION_ENABLED:  ret = on_operation_enabled(); break;
            case Cia402State::FAULT:              ret = on_fault(); break;
            default: break;
        }

        // 3. 处理状态跳转
        if (ret == SmRet::NEXT_STATE) {
            transition_to_next(); // 自动流转逻辑
        } else if (ret == SmRet::ERROR) {
            go_to_fault();
        }
        
        // 4. 更新 StatusWord
        update_status_word();
    }

    // 用户API：请求目标状态
    void request_target_state(Cia402State target) {
        // 这里不直接切状态，而是修改 ControlWord，由 tick() 中的 handle_control_word_request 决定如何切换
        target_state_request_ = target; 
    }

protected:
    // --- 必须由子类实现的虚函数 (非阻塞) ---
    virtual SmRet on_switch_on_disabled() = 0;
    virtual SmRet on_ready_to_switch_on() = 0;
    virtual SmRet on_switched_on() = 0;
    virtual SmRet on_operation_enabled() = 0;
    virtual SmRet on_fault() = 0;

    // --- 状态数据 ---
    uint16_t control_word_;
    uint16_t status_word_;
    Cia402State current_state_;
    Cia402State target_state_request_;
    int8_t operation_mode_; // 0x6060
};
```



| **状态 (State)**           | **CiA 402 定义** | **逻辑功能**                                | **主接触器 (DC)**     | **并网/负载接触器 (AC)** | **PWM**                           | **调制器**                                         | **观测器 / PLL**    | **ADC 校准**          | **跳转条件 (返回1)**         |
| -------------------------- | ---------------- | ------------------------------------------- | --------------------- | ------------------------ | --------------------------------- | -------------------------------------------------- | ------------------- | --------------------- | ---------------------------- |
| **(0) Not Ready**          | Init             | 硬件初始化                                  | 断开                  | 断开                     | Disable                           | 禁用                                               | 停止                | **执行一次**          | 初始化完成                   |
| **(1) Switch On Disabled** | Initialization   | **待机/低功耗** 复位故障 等待指令           | 断开                  | 断开                     | Disable                           | 禁用 (0输出)                                       | 停止                | **周期性执行** (推荐) | 用户请求 Switch On 且 无故障 |
| **(2) Ready to Switch On** | Pre-charge       | **预充电 & 同步** 建立母线电压 锁定电网相位 | **闭合** (预充完成后) | 断开                     | Disable                           | 禁用 (0输出)                                       | **运行 (开始锁相)** | 禁止                  | $V_{dc}$ 正常 且 **PLL锁定** |
| **(3) Switched On**        | Ready to Run     | **物理连接建立** 闭合AC继电器 准备发波      | 闭合                  | **闭合**                 | **Enable** (但在死区状态或零矢量) | **使能** (输出与电网电压匹配的占空比，保持电流为0) | 运行 (已锁定)       | **严禁**              | 继电器动作稳定 且 软启就绪   |
| **(4) Op. Enabled**        | Run              | **闭环运行** 执行电流/电压环                | 闭合                  | 闭合                     | **Enable** (正常发波)             | **使能** (正常调制)                                | 运行                | 禁止                  | (保持运行，返回0)            |
| **(5) Fault**              | Fault            | **故障停机** 封波、断触点                   | 断开 (或根据故障等级) | **立即断开**             | **立即Disable**                   | 禁用                                               | 停止                | 禁止                  | 用户请求 Reset (返回1)       |



### 四、 实施细节上的关键注意事项

#### 1. "Request State" 的处理逻辑 (sm_dispatch)

您提到“向前必须一级级推进，向后可以直接退回”，这是非常正确的安全逻辑。

- **向前 (Forward)**: 如果当前是 `Switch On Disabled`，用户直接请求 `Operation Enabled`，调度器**不能**直接切过去。它应该将 `ControlWord` 设为最终目标，但在每一帧 `tick` 中，状态机只能从 1->2，下一帧 2->3，再 3->4。这保证了预充、锁相的物理过程不被跳过。
- **向后 (Backward)**: 任何时候发生 Fault 或用户请求 Stop，必须允许从 4 直接跳回 1 或 2（通常是跳回 Switch On Disabled 或 Ready to Switch On），中间不需要经过 3。

#### 2. PWM 使能与继电器闭合的配合 (状态 3 的细节)

在 `Switched On` 阶段，存在两种常见的并网策略，需要在子类中明确：

- **策略 A (零电流合闸)**: PWM 封锁 -> 闭合 AC 继电器 -> 开启 PWM。
  - 优点：控制简单。
  - 缺点：继电器闭合瞬间可能有冲击电流（因为电网电压直接加在 LCL 滤波电容上）。
- **策略 B (软合闸)**: 开启 PWM -> 控制输出电压跟踪电网电压 (电流为0) -> 闭合 AC 继电器 -> 进入电流闭环。
  - 优点：无冲击，平滑。
  - 缺点：要求 PLL 极其精准，对电压传感器的精度要求高。
  - **架构适配**: 您的设计更适合策略 B，因为您在 Step 3 已经使能了调制器。但务必注意，**PLL必须在 Step 2 就准备好**。

#### 3. 虚函数的返回值处理

建议虚函数不仅返回状态跳转指令，最好能通过引用参数或成员变量反馈**错误码**。

- 例如：`(2) Ready to Switch On` 返回 -1 (Error)，同时设置 `error_code = PRECHARGE_TIMEOUT`。这对于排查“为什么卡在预充过不去”非常重要。

#### 4. 安全管理模块 (Safety Monitor)

您提到安全模块始终运行，这是对的。

- **建议实现**：安全检测（过压、过流、急停）应当在 `tick()` 函数的最开始执行，优先级高于状态流转。如果安全检测触发，直接强行覆写 `next_state = FAULT`，忽略任何用户请求。

这个设计方案在工程实践中是非常扎实的，具有很好的扩展性和维护性。





### 1. 为什么不能没有延迟？

如果您在状态机中仅依靠逻辑判断（例如：发指令闭合继电器 -> 下一行代码立刻检查辅助触点 -> 通过即跳转），会发生灾难：

- **触点抖动（Contact Bounce）**：机械继电器闭合瞬间，触点会物理弹跳 5ms~20ms。如果此时您的状态机已经跳到了“发波”状态，IGBT 开始高频开关，而继电器触点还在弹跳，会产生**严重的拉弧（Arcing）**，甚至瞬间烧毁触点或炸机。
- **模拟量滞后**：ADC 采样通常有低通滤波器（无论是 RC 硬件还是滑动平均软件滤波）。物理电压建立了，但 MCU 看到的变量值可能滞后 10~50ms。
- **电网暂态**：继电器闭合瞬间，会引起电网电压的微小跌落或振荡，此时 PLL 需要时间重新稳定。

### 2. 各个环节的延迟策略

建议在您的 `tick()` 函数或虚函数实现中，引入非阻塞的计时器（Timer）。

#### A. Switch On Disabled $\to$ Ready to Switch On (预充电环节)

- **物理动作**：闭合预充继电器。
- **建议延迟**：**不需要固定延迟，但需要“稳定判定”。**
- **逻辑**：
  1. 检测 $V_{dc} > V_{target}$。
  2. **保持逻辑**：$V_{dc}$ 必须**连续** 50ms~100ms 高于阈值，才认为预充完成。
  3. **原因**：防止电网电压波动导致瞬间采样值达标，实际上电容还没充实。

#### B. Ready to Switch On $\to$ Switched On (最关键！机械动作)

- **物理动作**：闭合主继电器 / 并网继电器。
- **建议延迟**：**必须强制增加 50ms ~ 200ms 的去抖动时间**。
- **逻辑**：
  1. 发出 `Close_Relay` 指令。
  2. 进入一个内部子状态 `WAIT_RELAY_SETTLE`。
  3. 即使反馈信号（辅助触点）已经闭合，也**强制等待** 100ms。
  4. 时间到后，再次检查反馈信号，如果通过，返回 `NEXT`。
- **原因**：这是为了保护继电器和 IGBT。**这是全流程中最需要“死延时”的地方。**

#### C. Switched On $\to$ Operation Enabled (电气动作)

- **物理动作**：PWM 使能。
- **建议延迟**：**极短 (1ms - 10ms) 或 无延迟**。
- **逻辑**：
  1. 只要上一步（Switched On）的机械去抖动做得足够好，这一步通常不需要额外延时。
  2. 但是，如果是**软启动**（PWM 从 0 占空比开始），需要给调制器一个复位/初始化的时间（比如 1 个 PWM 周期）。
- **特殊情况**：如果您在这一步才闭合负载接触器（Load Contactor），则参考 B，必须加 100ms。



1. **Fault 状态**：无延迟，立即响应（中断级或硬件级）。
2. **向前逻辑（Forward）**：
   - **涉及机械触点吸合**：**必须**加 100ms 左右的去抖延时。
   - **涉及模拟量判断（电压/锁相）**：**建议**加“连续 N 个周期满足条件”的逻辑（相当于数字滤波延时）。
   - **纯软件状态切换**：不需要延时。
3. **向后逻辑（Shutdown）**：
   - **正常停机**：通常不需要延时，或者遵循软关断斜坡。
   - **切断继电器**：断开指令发出后，通常不需要软件延时，物理断开的电弧由灭弧室处理。

**结论**：在状态机中增加一个 **`state_timer`** 成员变量，并在机械动作相关的状态（2->3）强制检查这个计时器，是非常必要且专业的做法。



example

### 小容量并网系统的 CiA 402 最佳实践流

您可以直接把这个流程写到您的代码注释里：

1. **上电 (Init)**: 硬件初始化，ADC 校准。StatusWord = 0。
2. **进入 Switch On Disabled**: 此时 PWM 封锁，继电器断开。StatusWord = 0x0250。
   - *动作*: 持续监测电网电压、频率、温度。
3. **接收命令 0x0006 (Shutdown)**:
   - *动作*: **执行预充电流程**。
   - *完成*: $V_{dc}$ 建立，PLL 锁定。
   - *跳转*: 状态变为 `Ready to Switch On`。StatusWord = 0x0231。
4. **接收命令 0x0007 (Switch On)**:
   - *动作*: **闭合 AC 并网继电器**。
   - *完成*: 物理连接建立。
   - *跳转*: 状态变为 `Switched On`。StatusWord = 0x0233。
5. **接收命令 0x000F (Enable Operation)**:
   - *动作*: **开启 PWM**，电流环 Soft-Start (从0缓慢增加)。
   - *完成*: 进入闭环控制。
   - *跳转*: 状态变为 `Operation Enabled`。StatusWord = 0x0237。



### 4. 参数识别 (Motor/Grid ID)

- **位置**: **必须在 `Operation Enabled` 状态下进行**。
- **逻辑**: 参数识别需要发波（PWM），这违反了其他所有状态的定义。
- **如何实现**:
  1. CiA 402 定义了一个对象字典 **Modes of Operation (0x6060)**。
  2. 正常运行时，Mode = 1 (Profile Position) 或 3 (Profile Velocity) 或 4 (Torque Profile)。
  3. 需要参数识别时，您可以自定义一个模式，例如 **Mode = -1 (Manufacturer Specific: Auto-Tuning)**。
  4. 流程：
     - 设置 `Modes of Operation` = -1。
     - 按照正常流程一路走到 `Operation Enabled`。
     - 因为模式是 -1，电流环不执行正常的 FOC，而是执行“注入高频信号”或“直流脉冲”逻辑。
     - 识别完成后，自动通过 StatusWord 告知上位机，或者自动跳回 `Switched On`。
