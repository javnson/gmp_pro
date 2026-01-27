# GMP CTL 组件库（Component Library）完整指南

## 📚 目录

1. [概述](#概述)
2. [模块架构](#模块架构)
3. [各模块详细说明](#各模块详细说明)
4. [API 命名规范](#api-命名规范)
5. [快速使用指南](#快速使用指南)
6. [自定义功能模块开发](#自定义功能模块开发)
7. [常见集成场景](#常见集成场景)

---

## 概述

GMP CTL 组件库（Control Template Library）是一套模块化、可复用的 C 语言控制系统库，为数字控制应用的快速开发提供了大量经过验证的、独立的控制模块。

### 核心特性

- ✅ **模块化设计**: 所有模块独立实现，可自由组合
- ✅ **统一接口**: 遵循严格的命名规范和接口规范
- ✅ **易于集成**: 无复杂依赖，支持多平台
- ✅ **完整文档**: 每个模块都有详细的头文件注释（Doxygen）
- ✅ **实测验证**: 在多个项目中成功应用

### 适用场景

- 永磁同步电机（PMSM）矢量控制
- 异步电机（ACM）矢量控制
- 数字电源（Buck、Boost、逆变器等）
- 电网互动系统（并网逆变器）
- 运动控制和伺服系统
- 其他嵌入式数字控制应用

---

## 模块架构

```
ctl/component/
├── interface/              # 接口模块：ADC、PWM、DAC通道抽象
├── intrinsic/             # 基础模块：PID、滤波器、信号生成等
├── motor_control/         # 电机控制：编码器、电流环、速度环、观测器等
├── digital_power/         # 数字电源：Buck/Boost拓扑、三相逆变器等
├── hardware_preset/       # 硬件预定义：电机、变流器、传感器参数
└── dsa/                   # 动态信号分析：数据采集、频率分析等
```

### 模块关系图

```
Application Layer (应用层)
  ↓
Motor Control Controllers (电机控制器)
  ├─ PMSM Controllers
  ├─ ACM Controllers
  └─ Digital Power Controllers
  ↓
Core Algorithm Modules (核心算法)
  ├─ Current Loops (电流环)
  ├─ Motion Control (运动控制)
  ├─ Observers (观测器)
  └─ Consultant (参数计算)
  ↓
Intrinsic Modules (基础模块)
  ├─ PID Controllers
  ├─ Filters
  ├─ Signal Generators
  └─ Protection
  ↓
Interface Modules (接口层)
  ├─ ADC Channels
  ├─ PWM Channels
  └─ DAC Channels
  ↓
Hardware (硬件层)
```

---

## 各模块详细说明

### 1. 接口模块（Interface）

**位置**: `ctl/component/interface/`

**功能**: 提供硬件外设与控制算法之间的标准化接口

#### 1.1 ADC 通道接口

```c
// 文件: adc_channel.h, adc_ptr_channel.h

// 结构体示例
typedef struct {
    ctrl_gt gain;          // ADC增益（用于转换为标幺值）
    ctrl_gt bias;          // ADC偏置（用于补偿零点漂移）
    ctrl_gt value;         // 采样值（标幺值）
    ctrl_gt raw;           // 原始采样值
    adc_channel_port_t control_port;  // 控制端口
} adc_channel_t;

// 典型使用流程
void adc_init() {
    // 初始化ADC通道
    ctl_init_adc_channel(&adc_ia, gain, bias, fs);
}

void adc_isr() {
    // 在ISR中调用，自动执行ADC采样→p.u.转换
    ctl_step_adc_channel(&adc_ia, raw_adc_value);
}
```

**关键特性**:
- ✅ 支持增益和偏置补偿
- ✅ 自动转换为标幺值（p.u.）
- ✅ 支持批量三相采样（`tri_ptr_adc_channel`）

#### 1.2 PWM 通道接口

```c
// 文件: pwm_channel.h

typedef struct {
    ctrl_gt compare_value;     // PWM占空比（0-1）
    pwm_channel_port_t control_port;
} pwm_channel_t;

// 使用示例
void pwm_init() {
    ctl_init_pwm_channel(&pwm_phase_u, min, max);
}

void pwm_update(ctrl_gt duty) {
    ctl_set_pwm_channel_duty(&pwm_phase_u, duty);
}
```

**关键特性**:
- ✅ 支持占空比（0-1）直接设置
- ✅ 自动处理死区时间
- ✅ 支持多路同步PWM

#### 1.3 DAC 通道接口

```c
// 文件: dac_channel.h

typedef struct {
    ctrl_gt value;           // DAC输出值
    dac_channel_port_t control_port;
} dac_channel_t;

// 使用
ctl_step_dac_channel(&dac_ch0, value);
```

#### 1.4 PWM 调制器接口

```c
// 文件: pwm_modulator.h

// SVPWM调制器
typedef struct {
    abc_value_t vab0_out;    // αβ坐标系输出
    // 内部计算产生三相PWM占空比
} spwm_modulator_t;

void modulator_init() {
    ctl_init_spwm_modulator(&spwm, 
        PWM_CMP_MAX,         // PWM计数最大值
        DEADBAND,            // 死区
        &adc_iabc,           // 电流反馈（用于过调制）
        &udc                 // 直流母线电压
    );
}

// 在控制循环中调用
ctl_step_svpwm_modulator(&spwm);
```

---

### 2. 基础模块（Intrinsic）

**位置**: `ctl/component/intrinsic/`

**功能**: 数字控制的基础构建块

#### 2.1 基本模块（Basic）

| 模块 | 功能 | 应用 |
|------|------|------|
| **Divider** | 频率分频器 | 在快速ISR中执行低频任务 |
| **Saturation** | 限幅器 | 约束信号在指定范围内 |
| **Slope Limiter** | 斜率限制器 | 限制信号变化速率 |
| **Hysteresis** | 滞后控制器 | Bang-Bang 控制（如电流控制） |

```c
// 频率分频器示例
typedef struct {
    uint16_t period;
    uint16_t counter;
} frequency_divider_t;

ctl_init_frequency_divider(&div, 100);  // 分频100倍
if (ctl_is_frequency_divider_match(&div)) {
    // 每100个周期执行一次
    slow_task();
}
ctl_step_frequency_divider(&div);
```

#### 2.2 连续控制器（Continuous）

```c
// PID 控制器
typedef struct {
    ctrl_gt kp, ki, kd;      // P, I, D 增益
    ctrl_gt error_integral;  // 积分项
    ctrl_gt error_prev;      // 前一周期误差（用于微分）
} continuous_pid_t;

void pid_init() {
    ctl_init_continuous_pid(&pid, kp, ki, kd, 
                           min_output, max_output,
                           sampling_time);
}

ctrl_gt pid_step(ctrl_gt ref, ctrl_gt feedback) {
    return ctl_step_continuous_pid(&pid, ref, feedback);
}
```

#### 2.3 离散控制器（Discrete）

```c
// 离散PID
typedef struct {
    // Z域形式系数
    ctrl_gt a0, a1, a2;
    ctrl_gt b0, b1, b2;
    // 状态寄存器
} discrete_pid_t;

// 比连续PID计算更高效
```

#### 2.4 滤波器和补偿器

```c
// IIR 滤波器
typedef struct {
    // 分子系数 (Numerator)
    ctrl_gt b[3];  // b0, b1, b2
    // 分母系数 (Denominator)
    ctrl_gt a[3];  // a1, a2 (a0 = 1)
    // 状态
    ctrl_gt x[2];  // x(n-1), x(n-2)
    ctrl_gt y[2];  // y(n-1), y(n-2)
} iir_filter_t;

// 极点-零点补偿器（1P1Z, 2P2Z, 3P3Z）
```

#### 2.5 信号生成器（Signal Generators）

```c
// 正弦波生成器（高效旋转器实现）
typedef struct {
    ctrl_gt sin_val, cos_val;  // 当前的sin和cos值
    ctrl_gt inc_sin, inc_cos;  // 增量（频率相关）
} sine_generator_t;

void sine_init() {
    // 初始化频率为 freq_hz
    ctl_init_sine_generator_via_freq(&gen, 50.0, SAMPLE_FREQ);
}

void sine_step() {
    ctl_step_sine_generator(&gen);
    sin_out = gen.sin_val;
    cos_out = gen.cos_val;
}
```

#### 2.6 保护模块（Protection）

```c
// 通用边界保护监控
typedef struct {
    ctrl_gt max_limit;
    ctrl_gt min_limit;
    uint32_t fault_count;      // 故障计数
    uint32_t fault_threshold;  // 故障判定阈值
} protection_monitor_t;

if (ctl_is_protection_triggered(&monitor)) {
    // 触发保护动作
    error_handler();
}
```

---

### 3. 电机控制模块（Motor Control）

**位置**: `ctl/component/motor_control/`

**功能**: 电机矢量控制的所有核心算法

#### 3.1 基本模块（Basic）

##### 3.1.1 编码器处理

```c
// 单转绝对值编码器
typedef struct {
    motor_rotation_ift rotation_if;  // 旋转接口（位置、速度）
    // ...内部状态
} pos_absolute_encoder_t;

void encoder_init() {
    ctl_init_pos_absolute_encoder(&enc, 
        FULL_SCALE,           // 编码器满度值
        POLE_PAIRS,           // 极对数
        SAMPLE_FREQ);
}

// 自动计数编码器（增量式转绝对）
typedef struct {
    pos_autoturn_encoder_t enc;
    // ...
} auto_turn_t;

ctl_init_autoturn_pos_encoder(&enc, POLE_PAIRS, SAMPLE_FREQ);
```

##### 3.1.2 编码器校准

```c
// 绝对值编码器电气零点自动校准
typedef struct {
    enum {
        CALIBRATE_IDLE,
        CALIBRATE_INJECT,
        CALIBRATE_MEASURE,
        CALIBRATE_DONE
    } state;
    // ...
} encoder_calibrate_t;

void calibrate_init() {
    ctl_init_encoder_calibrate(&cal, 
        INJECTION_CURRENT,   // 注入d轴电流
        SAMPLE_FREQ);
}

// 状态机自动进行
ctl_step_encoder_calibrate(&cal, i_q_feedback);
```

##### 3.1.3 SVPWM 生成

```c
// 空间矢量PWM调制
typedef struct {
    abc_value_t vab0_out;      // αβ坐标系
    // ...内部计算
} svpwm_modulator_t;

void svpwm_init() {
    ctl_init_svpwm_modulator(&svpwm,
        PWM_CMP_MAX,
        DEADBAND,
        &current_adc,
        &voltage_adc);
}

// 在控制循环中调用
ctl_step_svpwm_modulator(&svpwm);
```

##### 3.1.4 坐标变换

```
// Clarke 变换（三相→αβ）
[Iα, Iβ] = Clarke[Ia, Ib, Ic]

// Park 变换（αβ→dq）
[Id, Iq] = Park(Iα, Iβ, θ)

// 反向 Park 变换（dq→αβ）
[Vα, Vβ] = Park_inv(Vd, Vq, θ)

// 反向 Clarke 变换（αβ→三相）
[Va, Vb, Vc] = Clarke_inv(Vα, Vβ)
```

##### 3.1.5 电压解耦

```c
// PMSM 去耦合控制
typedef struct {
    // 计算前馈补偿电压
    // Vd_ff = -ωLq·Iq
    // Vq_ff = ωLd·Id + ω·Ψf
    ctrl_gt lq, ld;          // Lq, Ld 电感
    ctrl_gt rs;              // 电阻
    ctrl_gt psi_f;           // 永磁磁链
} decouple_pmsm_t;

ctl_step_decouple_pmsm(&dec, i_d, i_q, w_m);
```

##### 3.1.6 V/F 曲线生成（开环控制）

```c
// 恒压恒频生成
typedef struct {
    ctrl_gt v_cmd;           // 电压给定
    ctrl_gt f_cmd;           // 频率给定
} vf_generator_t;

// 使用斜坡生成平滑启动
ctl_init_const_slope_f_controller(&ramp, 
    target_freq,             // 目标频率（Hz）
    freq_slope,              // 频率斜坡（Hz/s）
    SAMPLE_FREQ);
```

#### 3.2 电流控制器（Current Loop）

##### 3.2.1 FOC 电流控制器

```c
// 场定向控制（FOC）核心
typedef struct {
    // PI 控制器
    continuous_pid_t pid_id;
    continuous_pid_t pid_iq;
    
    // 输入：电流给定和反馈
    ctrl_gt id_ref, iq_ref;
    ctrl_gt id, iq;
    
    // 输出：电压给定
    ctrl_gt vd_cmd, vq_cmd;
    
    // 电机参数
    ctrl_gt ld, lq, rs;
    ctrl_gt psi_f;           // 永磁磁链
    ctrl_gt w_m;             // 机械角速度
} mtr_current_ctrl_t;

void current_ctrl_init() {
    mtr_current_init_t init = {
        .fs = SAMPLE_FREQ,
        .v_base = VOLTAGE_BASE,
        .i_base = CURRENT_BASE,
        .spd_base = SPEED_BASE,
        .pole_pairs = POLE_PAIRS,
        .mtr_Rs = MOTOR_RS,
        .mtr_Ld = MOTOR_LD,
        .mtr_Lq = MOTOR_LQ,
    };
    
    // 自动调谐PI参数
    ctl_auto_tuning_mtr_current_ctrl(&init);
    ctl_init_mtr_current_ctrl(&mtr_ctrl, &init);
}

// 每个控制周期调用一次
void current_ctrl_step() {
    // 设置电流给定
    ctl_set_mtr_current_ctrl_ref(&mtr_ctrl, id_ref, iq_ref);
    
    // 执行电流控制
    ctl_step_current_controller(&mtr_ctrl);
    
    // 获取输出电压
    vd_out = mtr_ctrl.vd_cmd;
    vq_out = mtr_ctrl.vq_cmd;
}
```

##### 3.2.2 电流参考生成（Current Reference Generation）

**MTPA（最大转矩/安培）**
```c
// 用于高效运行（< 基速）
// 最优电流分布：最大化转矩/电流比
typedef struct {
    // Id 和 Iq 的最优关系
    // 对凸极电机：负d轴电流可减少铜损
} mtpa_t;

i_d_opt = ctl_get_mtpa_id(motor_speed);
i_q_opt = ctl_get_mtpa_iq(torque_cmd);
```

**Field Weakening（弱磁）**
```c
// 用于高速运行（> 基速）
// 注入负d轴电流以降低反电动势
typedef struct {
    ctrl_gt u_lim;           // 电压限制
    ctrl_gt i_lim;           // 电流限制
} mtpv_t;

// 自动计算 Id_weaken = f(speed, torque)
```

#### 3.3 运动控制（Motion Control）

##### 3.3.1 速度/位置控制器

```c
// 基本的速度和位置双环控制
typedef struct {
    // 速度环
    continuous_pid_t pid_spd;
    ctrl_gt spd_ref, spd_fb;
    ctrl_gt iq_cmd;          // 速度环输出→Iq给定
    
    // 位置环
    continuous_pid_t pid_pos;
    ctrl_gt pos_ref, pos_fb;
    ctrl_gt spd_cmd;         // 位置环输出→速度给定
} vel_pos_ctrl_t;

void motion_ctrl_init() {
    ctl_init_vel_pos_ctrl(&motion_ctrl,
        kp_spd, kp_pos,      // 速度和位置环P增益
        ki_spd, ki_pos,      // I增益
        spd_lim,             // 速度限制
        curr_lim,            // 电流限制
        spd_div,             // 速度计算分频系数
        pos_div,             // 位置计算分频系数
        SAMPLE_FREQ);
}

// 运动控制循环
void motion_step() {
    // 设置目标位置或速度
    ctl_set_vel_pos_cmd_pos(&motion_ctrl, pos_target);
    // 或者
    ctl_set_vel_pos_cmd_spd(&motion_ctrl, spd_target);
    
    // 执行运动控制
    ctl_step_vel_pos_ctrl(&motion_ctrl);
    
    // 输出：电流给定
    iq_ref = ctl_get_vel_pos_cmd(&motion_ctrl);
}
```

##### 3.3.2 轨迹生成

```c
// 梯形速度轨迹
typedef struct {
    // 加速段、匀速段、减速段
    ctrl_gt accel;
    ctrl_gt decel;
} trapezoidal_trajectory_t;

// S曲线轨迹（考虑加速度变化率）
typedef struct {
    ctrl_gt jerk;            // 加速度变化率
} s_curve_trajectory_t;

// 正弦轨迹（圆周运动）
typedef struct {
    sine_generator_t gen;
} sinusoidal_trajectory_t;
```

#### 3.4 观测器（Observer）

##### 3.4.1 PMSM 观测器

**Flux Observer（有位置传感器）**
```c
// 基于电压模型的定子磁链观测
typedef struct {
    // 通过积分得到磁链
    // Ψ = ∫(U - I·R)dt
    ctrl_gt psi_alpha, psi_beta;
} pmsm_flux_observer_t;
```

**Sliding Mode Observer（滑模观测器）**
```c
// 无传感器控制（中高速）
// 跟踪反电动势估计转子位置
typedef struct {
    // 不需要位置传感器
    // 工作范围：5%-100% 额定速度
} pmsm_smo_t;
```

**High Frequency Injection（高频注入）**
```c
// 无传感器低速控制
// 利用电机凸极性注入高频信号
typedef struct {
    // 工作范围：0%-5% 额定速度
} pmsm_hfi_t;
```

#### 3.5 参数估计（Parameter Estimation）

```c
// 在线电阻估计（MRAS）
typedef struct {
    // 基于电机参考模型
    // 自适应调整 Rs 估计值
} online_rs_estimator_t;

// 离线参数估计
typedef struct {
    // 标准的电机参数测量流程
} offline_estimation_t;
```

#### 3.6 参数计算助手（Consultant）

```c
// 这些模块简化了参数计算和单位转换

// PMSM 参数助手
typedef struct {
    // 电机额定功率、电压、电流等
    // 自动计算：
    //  - 基础参数（时间常数、转动惯量等）
    //  - PI控制器参数
    //  - 弱磁策略参数
} pmsm_consultant_t;

void init_using_consultant() {
    pmsm_consultant_t consultant = {
        .power_rated = 5000,        // 5kW
        .v_rated = 400,             // 400V
        .i_rated = 10,              // 10A
        .freq_rated = 100,          // 100Hz
        .pole_pairs = 2,
    };
    
    // 自动计算所有参数
    ctl_init_pmsm_consultant(&consultant);
    
    // 获取自动调谐的PI参数
    ctrl_gt kp = ctl_get_pmsm_consultant_pid_kp(&consultant);
    ctrl_gt ki = ctl_get_pmsm_consultant_pid_ki(&consultant);
}
```

---

### 4. 数字电源模块（Digital Power）

**位置**: `ctl/component/digital_power/`

**功能**: 数字电源控制和拓扑管理

#### 4.1 基本拓扑（Basic）

```c
// Buck 转换器
typedef struct {
    continuous_pid_t pid_vout;      // 输出电压控制
    ctrl_gt d_cmd;                  // 占空比命令
} buck_controller_t;

// Boost 转换器
typedef struct {
    continuous_pid_t pid_vin;       // 输入电流控制
    continuous_pid_t pid_vout;      // 输出电压控制
    ctrl_gt d_cmd;
} boost_controller_t;

// Buck-Boost 转换器
typedef struct {
    // 四个工作区自动切换
    ctrl_gt d_cmd;
} buckboost_controller_t;
```

#### 4.2 三相逆变器

```c
// 三相并网逆变器（GFL - Grid-Feeding）
typedef struct {
    // 电网电压同步
    pll_t pll;                      // 锁相环
    proportional_resonant_t pr_ctrl;  // PR控制器
    
    // 输出电流给定
    ctrl_gt i_d_ref, i_q_ref;
} three_phase_gfl_t;

// 三相独立逆变器（GFM - Grid-Forming）
typedef struct {
    // 自主生成电压
    sine_generator_t v_gen;
    
    // 内环电流控制
    continuous_pid_t pid_id, pid_iq;
} three_phase_gfm_t;
```

#### 4.3 保护策略

```c
// VIP 保护（电压、电流、功率）
typedef struct {
    protection_monitor_t v_monitor;
    protection_monitor_t i_monitor;
    protection_monitor_t p_monitor;
} protection_strategy_t;

// 过流保护（ITOC - Inverse Time Over Current）
typedef struct {
    // 三级保护：
    // 1. 瞬时动作（超高电流）
    // 2. 短延时（故障电流）
    // 3. 长延时（过载）
} itoc_protection_t;
```

---

### 5. 硬件预定义（Hardware Preset）

**位置**: `ctl/component/hardware_preset/`

**功能**: 预定义常用硬件的参数，加快配置速度

#### 5.1 电机预定义

```c
// 位置: hardware_preset/pmsm_motor/

// 示例：涛动 TYI_5008 电机
#include <ctl/component/hardware_preset/pmsm_motor/TYI_5008_KV335.h>

// 包含的参数：
// - 额定功率、电压、电流
// - 极对数、转子磁链
// - 定子电阻、电感
// - 转动惯量、摩擦系数
```

#### 5.2 变流器预定义

```c
// 位置: hardware_preset/inverter_3ph/

// 示例：GMP Helios 3Ph GaN 逆变器
#include <ctl/component/hardware_preset/inverter_3ph/GMP_Helios_3PhGaNInv_LV.h>

// 包含的信息：
// - 额定功率、电流
// - PWM 频率、死区时间
// - 散热和过流保护参数
```

#### 5.3 电流传感器预定义

```c
// 位置: hardware_preset/current_sensor/

// 示例：Infineon TLE4971 霍尔传感器
#include <ctl/component/hardware_preset/current_sensor/TLE4971A.h>

// 灵敏度、偏置、满度值等
```

---

### 6. 动态信号分析（DSA）

**位置**: `ctl/component/dsa/`

**功能**: 实时数据采集和分析

#### 6.1 触发采集（DSA Trigger）

```c
// 事件触发的多通道数据采集
typedef struct {
    // 触发条件
    enum {
        TRIGGER_NONE,
        TRIGGER_RISING,      // 上升沿
        TRIGGER_FALLING,     // 下降沿
        TRIGGER_LEVEL,       // 级别
    } trigger_type;
    
    ctrl_gt trigger_level;
    uint32_t pre_samples;    // 触发前采样数
    uint32_t post_samples;   // 触发后采样数
    
    // 数据缓冲
    ctrl_gt *buffer[MAX_CHANNELS];
    uint32_t buffer_index;
    fast_gt is_complete;     // 采集完成标志
} dsa_trigger_t;

void dsa_init() {
    ctl_init_dsa_trigger(&dsa,
        buffer,
        BUFFER_SIZE,
        TRIGGER_RISING,
        trigger_level);
}

// 在快速中断中调用
void dsa_step() {
    ctl_step_dsa_trigger(&dsa, signal);
}

// 检查采集是否完成
if (ctl_is_dsa_trigger_complete(&dsa)) {
    // 数据已保存在 buffer 中
    process_data();
}
```

#### 6.2 正弦波分析（Sine Analyzer）

```c
// 实时分析交流信号（幅值、频率、相位等）
typedef struct {
    // 使用 SOGI（二阶广义积分器）
    sogi_t sogi;
    
    // 计算得到
    ctrl_gt amplitude;       // 幅值
    ctrl_gt phase;           // 相位
    ctrl_gt frequency;       // 频率
    ctrl_gt thd;             // 谐波失真
} sine_analyzer_t;

void analyzer_init() {
    ctl_init_sine_analyzer(&analyzer, 
        BASE_FREQ,
        SAMPLE_FREQ);
}

void analyzer_step(ctrl_gt signal) {
    ctl_step_sine_analyzer(&analyzer, signal);
}

// 获取分析结果
ctrl_gt amp = ctl_get_sine_analyzer_amplitude(&analyzer);
```

---

## API 命名规范

GMP CTL 遵循统一的 API 命名规范，帮助开发者快速理解函数功能。

### 命名结构

```
ctl_<action>_<target_object>_via_<data_source>_<configuration>
```

### 动作（Action）

| 前缀 | 含义 | 调用时机 | 示例 |
|------|------|----------|------|
| `init` | 初始化 | 启动时，仅调用一次 | `ctl_init_pid` |
| `setup` | 配置 | 启动时，仅调用一次 | `ctl_setup_motor` |
| `step` | 执行一步 | 实时循环/ISR中 | `ctl_step_pid` |
| `get` | 获取参数 | 任何时间 | `ctl_get_pid_kp` |
| `set` | 设置参数 | 任何时间 | `ctl_set_pid_limit` |
| `clear` | 清零 | 重置时 | `ctl_clear_pid` |
| `is` | 判定标志 | 任何时间 | `ctl_is_error` |
| `attach` | 连接接口 | 初始化时 | `ctl_attach_motor_sensor` |
| `helper` | 辅助计算 | 算法内部 | `ctl_helper_transform` |

### 命名示例

**PID 控制器：**
```c
ctl_init_continuous_pid(...);           // 初始化
ctl_set_pid_limit(&pid, max, min);      // 设置限幅
ctl_step_continuous_pid(&pid, ref, fb); // 执行计算
kp = ctl_get_pid_kp(&pid);              // 获取参数
ctl_clear_pid(&pid);                    // 清零状态
```

**电机电流控制器：**
```c
ctl_init_mtr_current_ctrl(&ctrl, &init);
ctl_set_mtr_current_ctrl_ref(&ctrl, id, iq);
ctl_step_current_controller(&ctrl);
ctl_attach_mtr_current_ctrl_port(&ctrl, &sensor);
```

**信号生成器：**
```c
ctl_init_sine_generator_via_freq(&gen, 50, fs);
ctl_step_sine_generator(&gen);
sin_val = gen.sin_val;
cos_val = gen.cos_val;
```

---

## 快速使用指南

### 场景1：永磁同步电机矢量控制（FOC）

**需求：** 实现基于编码器的 PMSM 矢量控制

**集成步骤：**

```c
// 1. 硬件初始化
void hardware_init() {
    // ADC 通道
    ctl_init_adc_channel(&adc_ia, gain_ia, bias_ia, fs);
    ctl_init_adc_channel(&adc_ib, gain_ib, bias_ib, fs);
    
    // PWM 通道
    ctl_init_pwm_channel(&pwm_u, 0, PWM_MAX);
    ctl_init_pwm_channel(&pwm_v, 0, PWM_MAX);
    ctl_init_pwm_channel(&pwm_w, 0, PWM_MAX);
    
    // 编码器
    ctl_init_autoturn_pos_encoder(&encoder, POLE_PAIRS, fs);
}

// 2. 控制器初始化
void controller_init() {
    // 电机参数
    mtr_current_init_t init = {
        .fs = CONTROL_FREQ,
        .v_base = 48,
        .i_base = 100,
        .pole_pairs = POLE_PAIRS,
        .mtr_Rs = RS,
        .mtr_Ld = LD,
        .mtr_Lq = LQ,
    };
    
    // 自动调谐和初始化
    ctl_auto_tuning_mtr_current_ctrl(&init);
    ctl_init_mtr_current_ctrl(&motor_ctrl, &init);
    
    // 初始化 SVPWM 调制器
    ctl_init_spwm_modulator(&spwm, 
        PWM_PERIOD, 
        DEADBAND,
        &adc_ia,  // 电流反馈
        &adc_bus);
    
    // 初始化运动控制器
    ctl_init_vel_pos_ctrl(&motion_ctrl,
        1.0f, 1.0f,    // P 增益
        0.01f, 0.01f,  // I 增益
        1000,          // 速度限制 (rpm)
        10,            // 电流限制 (A)
        5, 5,          // 分频
        CONTROL_FREQ);
}

// 3. 控制周期（在 ADC 中断中）
void control_isr() {
    // 输入：采样 ADC
    ctl_step_adc_channel(&adc_ia, raw_ia);
    ctl_step_adc_channel(&adc_ib, raw_ib);
    
    // 处理编码器
    theta_m = read_encoder();
    ctl_step_autoturn_pos_encoder(&encoder, theta_m);
    
    // 执行运动控制
    ctl_step_vel_pos_ctrl(&motion_ctrl);
    iq_ref = ctl_get_vel_pos_cmd(&motion_ctrl);
    
    // 执行电流控制
    ctl_set_mtr_current_ctrl_ref(&motor_ctrl, 0, iq_ref);
    ctl_step_current_controller(&motor_ctrl);
    
    // 调制
    spwm.vab0_out.dat[0] = motor_ctrl.vd_cmd;
    spwm.vab0_out.dat[1] = motor_ctrl.vq_cmd;
    ctl_step_svpwm_modulator(&spwm);
    
    // 输出：更新 PWM
    ctl_step_pwm_channel(&pwm_u, spwm_u);
    ctl_step_pwm_channel(&pwm_v, spwm_v);
    ctl_step_pwm_channel(&pwm_w, spwm_w);
}
```

### 场景2：三相并网逆变器（GFL）

**需求：** 实现三相并网逆变器，注入恒定电流

```c
void gfl_inverter_init() {
    // PLL 初始化（同步到电网电压）
    ctl_init_pll(&pll, 50.0, fs);
    
    // PR 控制器（用于电流控制）
    ctl_init_proportional_resonant(&pr_ctrl,
        kp, kr,              // P 和谐振增益
        50.0,                // 谐振频率（Hz）
        fs);
}

void gfl_inverter_control() {
    // 1. 采样电网电压和逆变器电流
    u_grid = ctl_read_grid_voltage();
    i_inv = ctl_read_inverter_current();
    
    // 2. 更新 PLL（锁相）
    ctl_step_pll(&pll, u_grid);
    theta_grid = pll.angle;
    w_grid = pll.frequency;
    
    // 3. 电流 dq 变换
    i_alpha = (clark_transform(i_grid));
    i_d = park_transform(i_alpha, theta_grid);
    i_q = park_transform(i_beta, theta_grid);
    
    // 4. 设置电流给定（或从外部控制）
    i_d_ref = 0;             // 无无功
    i_q_ref = INJECT_CURRENT; // 注入有功
    
    // 5. PR 控制器计算电压
    u_d_cmd = ctl_step_proportional_resonant(&pr_ctrl, 
                                             i_d_ref - i_d);
    
    // 6. dq→αβ 变换并输出
    u_alpha = inv_park_transform(u_d_cmd, 0, theta_grid);
    u_beta = inv_park_transform(0, u_q_cmd, theta_grid);
    
    // 7. 调制
    ctl_step_svpwm_modulator(&spwm);
}
```

### 场景3：Buck 电源管理

**需求：** 48V → 12V Buck 转换器，恒定输出电压

```c
void buck_init() {
    buck_controller_t init = {
        .v_in = 48,
        .v_out = 12,
        .fs = 100e3,         // 100kHz 开关频率
        .v_ref = 12,
    };
    
    ctl_init_buck_controller(&buck, &init);
}

void buck_control() {
    // 采样
    v_out = ctl_read_output_voltage();
    
    // 反馈控制
    d_cmd = ctl_step_buck_controller(&buck, v_out);
    
    // 更新 PWM
    ctl_set_pwm_duty(&pwm_buck, d_cmd);
}
```

---

## 自定义功能模块开发

### 什么时候需要自定义模块？

- 标准库中没有现成的功能
- 需要针对特定硬件优化
- 需要实现专有算法
- 需要集成第三方控制策略

### 自定义模块开发指南

#### 第1步：定义模块数据结构

```c
// 文件：my_custom_controller.h

#ifndef _MY_CUSTOM_CONTROLLER_H_
#define _MY_CUSTOM_CONTROLLER_H_

#include <gmp_core.h>

// 数据结构定义
typedef struct {
    // 参数
    ctrl_gt param1;
    ctrl_gt param2;
    
    // 状态变量
    ctrl_gt state1;
    ctrl_gt state2;
    
    // 中间结果
    ctrl_gt output;
} my_custom_controller_t;

// 初始化结构体
typedef struct {
    ctrl_gt init_param1;
    ctrl_gt init_param2;
    // ... 初始化所需的参数
} my_custom_controller_init_t;

#endif
```

#### 第2步：实现核心函数

```c
// 文件：my_custom_controller.c

#include "my_custom_controller.h"

// ===== 必须实现 =====

// 初始化函数：只在启动时调用一次
GMP_NOINLINE
void ctl_init_my_custom_controller(
    my_custom_controller_t *this,
    const my_custom_controller_init_t *init)
{
    GMP_ASSERT_PTR(this);
    GMP_ASSERT_PTR(init);
    
    // 初始化参数
    this->param1 = init->init_param1;
    this->param2 = init->init_param2;
    
    // 初始化状态
    this->state1 = 0;
    this->state2 = 0;
    
    // 初始化输出
    this->output = 0;
}

// 步进函数：每个控制周期调用一次
GMP_STATIC_INLINE
ctrl_gt ctl_step_my_custom_controller(
    my_custom_controller_t *this,
    ctrl_gt input)
{
    GMP_ASSERT_PTR(this);
    
    // 算法核心计算
    ctrl_gt temp = ctl_mul(input, this->param1);
    
    // 状态更新
    this->state1 = temp;
    
    // 输出
    this->output = ctl_add(this->state1, this->state2);
    
    return this->output;
}

// ===== 可选实现 =====

// 清零函数：重置状态，保持参数
GMP_STATIC_INLINE
void ctl_clear_my_custom_controller(
    my_custom_controller_t *this)
{
    GMP_ASSERT_PTR(this);
    
    this->state1 = 0;
    this->state2 = 0;
    this->output = 0;
}

// 获取参数
GMP_STATIC_INLINE
ctrl_gt ctl_get_my_custom_controller_output(
    const my_custom_controller_t *this)
{
    GMP_ASSERT_PTR(this);
    return this->output;
}

// 设置参数
GMP_STATIC_INLINE
void ctl_set_my_custom_controller_param1(
    my_custom_controller_t *this,
    ctrl_gt value)
{
    GMP_ASSERT_PTR(this);
    this->param1 = value;
}
```

#### 第3步：编写单元测试

```c
// 文件：test_my_custom_controller.c

#include "my_custom_controller.h"
#include <stdio.h>

void test_basic_operation() {
    my_custom_controller_t ctrl;
    my_custom_controller_init_t init = {
        .init_param1 = float2ctrl(0.5),
        .init_param2 = float2ctrl(0.2),
    };
    
    // 初始化
    ctl_init_my_custom_controller(&ctrl, &init);
    
    // 执行
    ctrl_gt output = ctl_step_my_custom_controller(&ctrl, float2ctrl(1.0));
    
    // 验证
    printf("Output: %d (expected: ~%d)\n", output, float2ctrl(0.7));
    
    // 清零测试
    ctl_clear_my_custom_controller(&ctrl);
    output = ctl_step_my_custom_controller(&ctrl, float2ctrl(1.0));
    printf("After clear: %d\n", output);
}

int main() {
    test_basic_operation();
    return 0;
}
```

#### 第4步：集成到主工程

```c
// 在工程的某个头文件中添加
#include "my_custom_controller.h"

// 在使用中
my_custom_controller_t my_ctrl;

void main_init() {
    my_custom_controller_init_t init = { ... };
    ctl_init_my_custom_controller(&my_ctrl, &init);
}

void main_control_loop() {
    ctrl_gt output = ctl_step_my_custom_controller(&my_ctrl, input);
}
```

### 模块开发最佳实践

#### ✅ 遵循命名规范

```c
// 好的命名
ctl_init_my_filter(...)
ctl_step_my_filter(...)
ctl_set_my_filter_cutoff(...)
ctl_get_my_filter_output(...)

// 不好的命名
InitFilter(...)
filter_step(...)
set_cutoff_freq(...)
```

#### ✅ 参数检验

```c
GMP_STATIC_INLINE
ctrl_gt ctl_step_my_controller(my_controller_t *this, ctrl_gt input) {
    // 检查指针有效性
    GMP_ASSERT_PTR(this);
    
    // 如果需要，检查输入范围
    input = ctl_sat(input, MAX_LIMIT, MIN_LIMIT);
    
    // ... 计算
}
```

#### ✅ 使用 GMP 提供的类型和宏

```c
// 使用 ctrl_gt 进行所有控制计算（固定点数学）
ctrl_gt x = float2ctrl(0.5);
ctrl_gt y = ctl_mul(x, param);

// 使用 GMP 的定义宏
GMP_STATIC_INLINE  // 函数声明
GMP_NOINLINE       // 禁止内联（用于大函数）
GMP_ASSERT_PTR()   // 指针检验
```

#### ✅ 文档化

```c
/**
 * @brief 初始化自定义控制器
 * @param [out] this 控制器对象指针
 * @param [in] init 初始化参数结构体
 * @note 此函数必须在 ctl_step 调用前执行一次
 * @return void
 */
void ctl_init_my_controller(my_controller_t *this, 
                            const my_controller_init_t *init);

/**
 * @brief 执行控制器一步
 * @param [in,out] this 控制器对象指针
 * @param [in] input 输入值
 * @return 控制输出值
 */
ctrl_gt ctl_step_my_controller(my_controller_t *this, ctrl_gt input);
```

#### ✅ 性能考虑

```c
// 频繁调用的函数使用内联
GMP_STATIC_INLINE
ctrl_gt ctl_step_my_filter(my_filter_t *this, ctrl_gt input) {
    // 简单计算，使用内联提速
    return ctl_mul(input, this->gain);
}

// 复杂计算或初始化使用非内联
GMP_NOINLINE
void ctl_init_my_controller(my_controller_t *this,
                            const my_controller_init_t *init) {
    // 复杂的初始化逻辑
}
```

---

## 常见集成场景

### 场景4：PMSM 无位置传感器控制（SMO）

**需求：** 无编码器，使用滑模观测器估计位置

```c
void smo_init() {
    // 初始化滑模观测器
    ctl_init_pmsm_smo(&smo,
        LD, LQ, RS,          // 电机参数
        PSI_F,               // 永磁磁链
        fs);                 // 采样频率
}

void smo_control() {
    // 采样电流
    ctl_step_adc_channel(&adc_ia, raw_ia);
    
    // 执行滑模观测
    ctl_step_pmsm_smo(&smo, 
                      motor_ctrl.vd_cmd,
                      motor_ctrl.vq_cmd,
                      adc_ia.value);
    
    // 获取估计的位置和速度
    theta_est = smo.theta_m;
    w_est = smo.w_m;
    
    // 更新到电流控制器
    ctl_attach_mtr_current_ctrl_port(&motor_ctrl, 
                                     &estimated_encoder);
}
```

### 场景5：MTPA（最大转矩/安培）控制

**需求：** 最大化电机效率

```c
void mtpa_init() {
    // 初始化 MTPA
    ctl_init_current_distributor(&mtpa,
        &lut_id_iq,          // Id-Iq 查表
        POLE_PAIRS);
}

void mtpa_control() {
    // 根据目标转矩分配 Id 和 Iq
    ctl_step_current_distributor(&mtpa, torque_ref);
    
    id_ref = mtpa.id_out;
    iq_ref = mtpa.iq_out;
    
    // 输入到电流控制器
    ctl_set_mtr_current_ctrl_ref(&motor_ctrl, id_ref, iq_ref);
}
```

### 场景6：弱磁（MTPV）控制

**需求：** 扩展电机高速运行范围

```c
void mtpv_init() {
    ctl_init_mtpv(&mtpv,
        V_BASE, I_BASE,      // 基值
        SPD_BASE,
        W_BASE,
        LD, LQ, RS, PSI_F);  // 电机参数
}

void mtpv_control() {
    // 计算弱磁电流
    ctl_step_mtpv(&mtpv, 
                  speed_feedback,
                  torque_ref);
    
    id_ref = mtpv.id_out;  // 负d轴电流
    iq_ref = mtpv.iq_out;
}
```

---

## 总结与建议

### 选择合适的模块

| 任务 | 推荐模块 | 说明 |
|------|----------|------|
| **电机启动** | V/F 生成器 + 开环电流 | 简单快速 |
| **低速精确控制** | FOC + 编码器 | 高动态性能 |
| **高效运行** | MTPA | 在额定速度以下 |
| **高速运行** | 弱磁（MTPV） | 超过额定速度 |
| **无传感器** | SMO / HFI | SMO 用于中高速，HFI 用于低速 |
| **并网逆变** | GFL + PLL + PR | 标准并网方案 |
| **孤岛模式** | GFM + 内环电流控制 | 独立供电 |

### 开发流程

```
1. 理解需求
   ↓
2. 选择合适的标准模块
   ↓
3. 配置模块参数
   ↓
4. 集成到主程序
   ↓
5. 仿真验证
   ↓
6. 硬件在环（HIL）测试
   ↓
7. 实车测试
```

### 性能优化建议

- 使用定点数学（`ctrl_gt`）而非浮点数
- 在快速中断中使用内联函数
- 利用 GMP 的频率分频器执行低频任务
- 定期查看 Doxygen 文档了解更多模块细节

### 故障排除

| 症状 | 可能原因 | 解决方案 |
|------|----------|----------|
| 电机不转 | PWM 未使能 | 检查 `ctl_fast_enable_output()` |
| 电流振荡 | PI 参数不合适 | 使用 `ctl_auto_tuning_*` 自动调参 |
| 位置跳变 | 编码器接触不良 | 检查硬件连接，使用滤波 |
| 计算超时 | 函数调用过多 | 使用频率分频器分散任务 |

---

## 进一步学习

- 查看 `ctl/component/motor_control/readme.md` 了解电机控制详情
- 查看 `ctl/component/intrinsic/readme.md` 了解基础模块
- 查看各模块的头文件注释（Doxygen 格式）
- 参考 `ctl/suite/` 中的完整示例项目

---

**文档版本**: 1.0  
**最后更新**: 2026-01-27  
**维护者**: GMP 开发团队
