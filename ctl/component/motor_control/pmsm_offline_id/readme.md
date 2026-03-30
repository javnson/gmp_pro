# PMSM 离线全自动参数辨识模块 (Offline Parameter Identification)

## 模块概述

`pmsm_offline_id` 是一个专为永磁同步电机（PMSM）设计的全自动、高精度离线参数辨识引擎。该模块作为底层 FOC 核心的“超级大脑（Supervisor）”，能够在电机初始上电或调试阶段，自动接管控制权，通过施加特定的电压/电流激励，精准测定电机的电气与机械参数。

本模块采用了**“数据采集与状态流转解耦”**的先进架构：

- **高频数据面 (ISR)**：利用双核时序器（State Sequencer），在中断中严格执行指令下发与高频数据记录（DSA），不涉及任何复杂逻辑判断。
- **低频控制面 (Background Loop)**：在后台循环中处理状态跳转、安全监控以及重算力的数学拟合（线性回归、物理积分），彻底杜绝中断超时与并发竞态。

------

## 核心功能与实现原理

本模块提供了四个核心识别子任务，用户可通过配置文件中的 `flag_enable_xxx` 自由开启或跳过。

### 1. 定子电阻与死区补偿测定 (RS_DT)

- **功能**：精准测量电机的相电阻 *R**s* (Ω) 以及逆变器系统的综合非线性死区压降 *V**co**m**p* (标幺值)。
- **实现原理**：
  - **六步寻位**：在电角度的 6 个不同位置施加直流偏置电流，消除转子位置带来的局部磁阻差异。
  - **阶梯注入**：在每个角度施加多组阶梯电流（如 0.05pu ~ 0.3pu）。
  - **线性拟合**：等待瞬态衰减后，记录稳态电流 *I* 与指令电压 *V*。在后台利用 DSA 引擎进行一元线性回归拟合公式 *V*=*R**s*⋅*I*+*V**co**m**p*。斜率即为纯粹的电阻 *R**s*，截距即为死区与管压降的综合补偿量。

### 2. 交/直轴电感测定 (LD_LQ)

- **功能**：测量 D 轴和 Q 轴的瞬态电感 *L**d* 和 *L**q* (H)，并可配置输出电感随电流偏置的饱和曲线。
- **实现原理**：
  - **微脉冲注入**：锁定转子后，冻结 PI 控制器，向目标坐标轴施加极短（如 2ms）的开环电压脉冲。
  - **物理积分法 (Integral Method)**：不使用容易受到高频噪声干扰的电流斜率 (*d**i*/*d**t*) 算法。相反，记录整个脉冲期间的高频电流响应曲线，在后台使用一阶惯性环节的积分方程 *τ*=*L*/*R* 求解时间常数，进而极其精准地提取电感值。
  - **母线补偿**：脉冲瞬间锁存真实母线电压 *U**d**c*，消除电网波动带来的计算误差。

### 3. 永磁体磁链测定 (FLUX)

- **功能**：测量转子永磁体磁链 *ψ**m* (Wb)，进而推算电机的真实 KV 值，并输出特征电流供弱磁控制使用。
- **实现原理**：
  - **PI 自动整定**：在启动前，模块会自动利用前置步骤测得的 *R**s* 和 *L**d*,*q*，通过零极点对消原则动态刷新 FOC 核心的 PI 参数，确保大动态拖拽时不失步。
  - **I/F 阶梯拖拽**：在 V/F 开环模式下施加恒定拖拽电流，使电机稳定在数个不同的目标转速下。
  - **反电势抗扰观测**：收集稳态电压、电流与转速。在计算反电势 *E* 时，**强行代入 RS_DT 阶段测出的死区压降 \*V\**co\**m\**p\* 进行补偿**。最后通过 ∣*E*∣ 对电角速度 *ω**e* 进行线性拟合，斜率即为磁链。

### 4. 机械参数测定 (MECH)

- **功能**：测量系统的总转动惯量 *J* (*k**g*⋅*m*2) 和黏性摩擦系数 *B* (*N*⋅*m*/(*r**a**d*/*s*))。
- **实现原理**：
  - **无扰切换 (Bumpless Transfer)**：电机在 I/F 开环模式下加速至低速阈值后，通过角度融合器（Angle Switcher）平滑切换至闭环控制。
  - **恒流加减速测试**：在稳态测定摩擦电流后，分别施加正向和反向的恒定转矩电流 (*I**q*)，记录速度的瞬态上升/下降曲线。
  - **动力学解耦**：利用 DSA 拟合出加速和减速段的角加速度 *α**a**cc* 和 *α**d**ec*，通过两组转矩方程联立，消去未知非线性摩擦力，精准解析出真实惯量 *J*。

------

## 模块集成与使用指南

### 1. 内存与实例分配

在系统的全局空间中实例化 OID 上下文，并为录波器分配专用的高频内存池。

C

```
ctl_pmsm_offline_id_t pmsm_oid;
#define DSA_BUFFER_SIZE 2048
ctrl_gt dsa_buffer[DSA_BUFFER_SIZE]; // 建议在 CCS 中使用 #pragma 分配到独立 RAM 段
```

### 2. 调度器接入 (System Hookup)

将 OID 的中断与后台函数分别接入系统控制流：

C

```
// 1. 接入高频中断 (例如 20kHz ADC 采样的 ISR 中)
// 必须放置在 ctl_step_foc_core(&mtr_ctrl) 执行之前！
ctl_step_pmsm_offline_id(&pmsm_oid);

// 2. 接入后台低优先级主循环 (While(1) 或 OS Task 中)
ctl_loop_pmsm_offline_id(&pmsm_oid);
```

### 3. 初始化与配置

在系统启动时，填充配置结构体并初始化状态机。可以自由开启或关闭指定的测试阶段。

C

```
ctl_pmsm_offline_id_init_t oid_cfg;
// ... 填充基准值、PWM 频率、极对数等 ...
oid_cfg.cfg_basic.flag_enable_rs_dt = 1;
oid_cfg.cfg_basic.flag_enable_ldq   = 1;
// ... 填充各子模块的激励电压、电流和测试时间 ...

ctl_init_pmsm_offline_id_sm(&pmsm_oid, &oid_cfg, dsa_buffer, DSA_BUFFER_SIZE);
```

### 4. 运行控制 (Lifecycle Control)

模块初始化后处于 `PMSM_OFFLINE_ID_READY` 状态，等待用户触发。

C

```
// 启动辨识：
ctl_enable_pmsm_offline_id(&pmsm_oid);

// 紧急停止/关闭输出：
ctl_disable_pmsm_offline_id(&pmsm_oid);

// 故障复位并返回就绪态：
ctl_clear_pmsm_offline_id(&pmsm_oid);
```

### 5. 获取结果

在主循环中监控状态机变量 `pmsm_oid.sm`。当其等于 `PMSM_OFFLINE_ID_COMPLETE` 时，电机将安全停机，此时可以直接从主控结构体中提取结果：

C

```
if (pmsm_oid.sm == PMSM_OFFLINE_ID_COMPLETE) {
    float rs_ohm = pmsm_oid.pmsm_param.Rs;
    float ld_henry = pmsm_oid.pmsm_param.Ld;
    float flux_weber = pmsm_oid.pmsm_param.flux_linkage;
    float deadtime_comp_pu = pmsm_oid.pmsm_param.V_comp_pu;
    // ...将结果写入 Flash 或用于后续 FOC 控制
}
```